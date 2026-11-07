"""WebRTC hub: brain is the single WebRTC peer for camera video.

Topology (brain terminates BOTH peer connections):

    smabo-app ══[WebRTC video]══> brain ──> vision pipeline (frames)
                                    └──[MediaRelay]──> smabo-web (preview ON)

The phone (smabo-app) is the only image source. It offers a send-only video
track to the brain; the brain answers, pulls frames for detection, and — only
while a web client has preview enabled — relays the same track onward to that
web client. The Vision tab draws detection rectangles itself as an HTML overlay
from ``/vision/detections``; the relayed video is therefore the raw pass-through
(no server-side re-encode).

Signaling rides the existing brain WebSocket relay (see relay.py):

    app → brain : /webrtc/offer        brain → app : /webrtc/answer
    app → brain : /webrtc/app_ice      (brain ICE is bundled in the answer SDP)
    web → brain : /webrtc/preview {on} brain → web : /webrtc/web_offer
    web → brain : /webrtc/web_answer
    web → brain : /webrtc/web_ice      (brain ICE is bundled in the offer SDP)

aiortc gathers ICE during setLocalDescription (non-trickle), so the brain's own
candidates travel inside the offer/answer SDP and only the remote peer trickles.

aiortc is imported lazily: if it (or PyAV) is not installed the hub disables
itself with a one-time warning and every handler becomes a no-op, so the relay
still serves eye control / sensors / config without camera video.
"""

import asyncio
import json
import logging

log = logging.getLogger(__name__)

try:
    from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate
    from aiortc.contrib.media import MediaRelay
    from aiortc.sdp import candidate_from_sdp
    HAVE_AIORTC = True
except Exception:  # pragma: no cover - import guard
    HAVE_AIORTC = False


# Host-candidate local preferences (RFC 8445 §5.1.2.1): preferred networks keep
# the maximum, every other interface gets less but is still offered.
_PREF_HIGH = 65535
_PREF_LOW = 32768


def prefer_ice_networks(networks) -> list:
    """Make ICE try host candidates on ``networks`` (CIDR strings) first.

    aioice gives every local interface the same priority and nominates the first
    pair whose check succeeds, so which path wins is a race. On Docker Desktop
    (macOS) that race is usually won by a path through Docker's NAT, which drops
    the flow after ~30 s ("Consent to send expired"), while the VM's own link to
    the Mac (vmnet, ``192.168.64.0/24``) is direct and stable. Ranking that
    network first makes the direct pair win. Other interfaces stay as fallbacks,
    so peers that can't reach the preferred network (a phone on the LAN) still
    connect; on hosts without such an interface nothing changes.

    Applies process-wide (aioice is patched once). Returns the parsed networks.
    """
    import ipaddress

    nets = []
    for n in networks or ():
        n = str(n).strip()
        if not n:
            continue
        try:
            nets.append(ipaddress.ip_network(n, strict=False))
        except ValueError:
            log.warning("WebRTC: ignoring bad ICE network %r", n)
    if not HAVE_AIORTC:
        return nets
    import aioice.ice as ice
    from aioice.candidate import candidate_priority

    orig = getattr(ice.Connection.get_component_candidates, "_smabo_orig",
                   ice.Connection.get_component_candidates)

    async def get_component_candidates(self, *args, **kwargs):
        candidates = await orig(self, *args, **kwargs)
        if nets:
            for c in candidates:
                if c.type != "host":
                    continue
                try:
                    addr = ipaddress.ip_address(c.host)
                except ValueError:
                    continue
                pref = _PREF_HIGH if any(addr in n for n in nets) else _PREF_LOW
                c.priority = candidate_priority(c.component, "host", local_pref=pref)
        return candidates

    get_component_candidates._smabo_orig = orig
    ice.Connection.get_component_candidates = get_component_candidates
    if nets:
        log.info("WebRTC: preferring ICE host candidates on %s",
                 ", ".join(str(n) for n in nets))
    return nets


def _to_bgr(frame):
    return frame.to_ndarray(format="bgr24")


def _parse_ice(d: dict):
    """Convert a browser/flutter ICE dict into an aiortc RTCIceCandidate."""
    cand_str = (d or {}).get("candidate") or ""
    if not cand_str:
        return None
    if cand_str.startswith("candidate:"):
        cand_str = cand_str[len("candidate:"):]
    try:
        c = candidate_from_sdp(cand_str)
    except Exception:
        return None
    c.sdpMid = d.get("sdpMid")
    c.sdpMLineIndex = d.get("sdpMLineIndex")
    return c


def _sdp_msg(topic: str, desc) -> dict:
    """rosbridge publish frame carrying an SDP (offer/answer) as JSON in data."""
    return {
        "op": "publish",
        "topic": topic,
        "msg": {"data": json.dumps({"sdp": desc.sdp, "type": desc.type})},
    }


class WebRtcHub:
    def __init__(self, *, process_frame, send_app, send_web):
        # process_frame(bgr_ndarray) -> awaitable : run one frame through vision
        # send_app(dict)             -> awaitable : broadcast a frame to app clients
        # send_web(ws, dict)         -> awaitable : send a frame to one web client
        self._process_frame = process_frame
        self._send_app = send_app
        self._send_web = send_web

        self._relay = MediaRelay() if HAVE_AIORTC else None
        self._app_pc = None             # peer with smabo-app (recvonly video)
        self._source = None             # incoming video track from app (to fan out)
        self._vision_task = None        # frame-pull loop feeding vision
        self._web_pcs = {}              # ws -> RTCPeerConnection (preview senders)
        self._preview_wanted = set()    # ws that requested preview (may predate stream)
        self._fps = 5.0
        self._warned = False

    # --------------------------------------------------------------- config
    @property
    def available(self) -> bool:
        return HAVE_AIORTC

    def set_fps(self, fps: float) -> None:
        try:
            self._fps = max(1.0, min(30.0, float(fps)))
        except (TypeError, ValueError):
            pass

    def _warn_once(self) -> None:
        if not self._warned:
            self._warned = True
            log.warning("aiortc is not installed; WebRTC camera/vision is disabled. "
                        "Run: pip install -r requirements.txt")

    # ----------------------------------------------------------- app ↔ brain
    async def handle_app_offer(self, sdp: dict) -> None:
        if not HAVE_AIORTC:
            self._warn_once()
            return
        await self.close_app()
        pc = RTCPeerConnection()
        self._app_pc = pc

        @pc.on("connectionstatechange")
        async def _on_state():
            if pc.connectionState in ("failed", "closed", "disconnected"):
                await self.close_app()

        @pc.on("track")
        def _on_track(track):
            if track.kind != "video":
                return
            self._source = track
            self._vision_task = asyncio.ensure_future(
                self._consume(self._relay.subscribe(track)))
            # Late-joining preview clients that asked before the stream existed.
            for ws in list(self._preview_wanted):
                asyncio.ensure_future(self._open_web_pc(ws))

        await pc.setRemoteDescription(
            RTCSessionDescription(sdp["sdp"], sdp["type"]))
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        await self._send_app(_sdp_msg("/webrtc/answer", pc.localDescription))
        log.info("WebRTC: app peer connected, answer sent")

    async def add_app_ice(self, d: dict) -> None:
        if not self._app_pc:
            return
        c = _parse_ice(d)
        if c is not None:
            try:
                await self._app_pc.addIceCandidate(c)
            except Exception:
                pass

    async def _consume(self, track) -> None:
        """Pull frames from the app track and feed vision at ~capture_fps."""
        loop = asyncio.get_running_loop()
        last = 0.0
        while True:
            try:
                frame = await track.recv()
            except Exception:
                break  # track ended / connection closed
            now = loop.time()
            if now - last < 1.0 / self._fps:
                continue  # drain but skip to throttle detection rate
            last = now
            try:
                # Pixel conversion is heavy: keep it off the event loop, which
                # also runs ICE. A loop stalled for seconds (slow CPU, emulated
                # Docker) misses the 0.5 s STUN consent deadlines and the peer
                # drops the connection ("Consent to send expired") after ~30 s.
                bgr = await loop.run_in_executor(None, _to_bgr, frame)
            except Exception:
                continue
            try:
                await self._process_frame(bgr)
            except Exception:
                log.exception("vision frame processing failed")

    async def close_app(self) -> None:
        if self._vision_task is not None:
            self._vision_task.cancel()
            self._vision_task = None
        if self._app_pc is not None:
            try:
                await self._app_pc.close()
            except Exception:
                pass
            self._app_pc = None
        self._source = None
        # The relayed source is gone, so tear down every preview sender too.
        # The clients stay in _preview_wanted: the next app track re-offers them.
        # (Detach first, so the "closed" events below don't count as the client
        # leaving — see _open_web_pc.)
        pcs = list(self._web_pcs.values())
        self._web_pcs.clear()
        for pc in pcs:
            try:
                await pc.close()
            except Exception:
                pass

    # ----------------------------------------------------------- web ↔ brain
    async def start_web_preview(self, ws) -> None:
        if not HAVE_AIORTC:
            self._warn_once()
            return
        self._preview_wanted.add(ws)
        log.info("WebRTC: preview requested by %s (stream %s)", ws,
                 "up" if self._source is not None else "not up yet")
        if self._source is not None:
            await self._open_web_pc(ws)

    async def _open_web_pc(self, ws) -> None:
        if self._source is None:
            return
        await self.stop_web_preview(ws, forget=False)
        pc = RTCPeerConnection()
        self._web_pcs[ws] = pc
        pc.addTrack(self._relay.subscribe(self._source))

        @pc.on("connectionstatechange")
        async def _on_state():
            log.info("WebRTC: preview %s: %s", ws, pc.connectionState)
            if self._web_pcs.get(ws) is not pc:
                return  # replaced or torn down on purpose: not the client leaving
            if pc.connectionState in ("failed", "closed", "disconnected"):
                await self.stop_web_preview(ws)

        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        await self._send_web(ws, _sdp_msg("/webrtc/web_offer", pc.localDescription))

    async def handle_web_answer(self, ws, sdp: dict) -> None:
        pc = self._web_pcs.get(ws)
        if pc is not None:
            try:
                await pc.setRemoteDescription(
                    RTCSessionDescription(sdp["sdp"], sdp["type"]))
            except Exception:
                pass

    async def add_web_ice(self, ws, d: dict) -> None:
        pc = self._web_pcs.get(ws)
        if pc is None:
            return
        c = _parse_ice(d)
        if c is not None:
            try:
                await pc.addIceCandidate(c)
            except Exception:
                pass

    async def stop_web_preview(self, ws, *, forget: bool = True) -> None:
        if forget:
            self._preview_wanted.discard(ws)
        pc = self._web_pcs.pop(ws, None)
        if pc is not None:
            try:
                await pc.close()
            except Exception:
                pass

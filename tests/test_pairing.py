"""Pairings: codes, tokens, hashes on disk, limits."""

import json
import os
import stat

import pytest

from brain.pairing import CODE_TTL_S, MAX_FAILURES, PairingError, Pairings, normalize_code, sha256


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def store(tmp_path):
    clock = Clock()
    announced = []
    p = Pairings.load(tmp_path / "brain" / "pairings.json", clock=clock, on_bootstrap=announced.append)
    p.announced = announced
    p.tick = clock
    return p


def test_first_browser_pairs_with_the_logged_code(store):
    code = store.ensure_bootstrap()
    assert store.announced == [code]
    done = store.pair(code.lower().replace("-", " "), name="PC1 Chrome")
    assert done.pairing.role == "web" and done.pairing.name == "PC1 Chrome"
    assert done.relay_key == ""
    assert store.check(done.token) is done.pairing
    # once a browser is paired there is no bootstrap code any more
    assert store.ensure_bootstrap() is None


def test_codes_work_once(store):
    code = store.ensure_bootstrap()
    store.pair(code)
    with pytest.raises(PairingError) as e:
        store.pair(code)
    assert e.value.reason == "wrong_code"


def test_expired_bootstrap_is_refused_and_replaced(store):
    code = store.ensure_bootstrap()
    store.tick.t += CODE_TTL_S + 1
    with pytest.raises(PairingError) as e:
        store.pair(code)
    assert e.value.reason == "expired"
    assert len(store.announced) == 2 and store.announced[1] != code


def test_wrong_codes_cancel_every_open_code(store):
    boot = store.ensure_bootstrap()
    for _ in range(MAX_FAILURES):
        with pytest.raises(PairingError):
            store.pair("AAAA-AAAA")
    with pytest.raises(PairingError):
        store.pair(boot)
    assert len(store.announced) == 2


def test_device_invite_gives_a_relay_key_and_checks_the_role(store):
    store.pair(store.ensure_bootstrap())
    code, ttl = store.invite("app", name="iPhone")
    assert ttl == CODE_TTL_S
    with pytest.raises(PairingError) as e:
        store.pair(code, role="esp32")
    assert e.value.reason == "wrong_role"
    code, _ = store.invite("app", name="iPhone")
    done = store.pair(code, role="app")
    assert done.pairing.role == "app" and done.relay_key
    assert done.pairing.relay_key == done.relay_key


def test_same_name_gets_a_number(store):
    store.pair(store.ensure_bootstrap(), name="iPhone")
    assert store.pair(store.invite("app")[0], name="iPhone").pairing.name == "iPhone 2"
    assert store.pair(store.invite("app")[0], name="iPhone").pairing.name == "iPhone 3"


def test_file_keeps_hashes_only_and_is_private(store):
    done = store.pair(store.ensure_bootstrap())
    raw = store.path.read_text()
    assert done.token not in raw
    assert json.loads(raw)["pairings"][0]["token_sha256"] == sha256(done.token)
    assert stat.S_IMODE(os.stat(store.path).st_mode) == 0o600
    again = Pairings.load(store.path)
    assert again.check(done.token).id == done.pairing.id


def test_broken_file_stops_the_brain(tmp_path):
    p = tmp_path / "pairings.json"
    p.write_text("{")
    with pytest.raises(SystemExit):
        Pairings.load(p)


def test_last_browser_cannot_be_removed(store):
    a = store.pair(store.ensure_bootstrap()).pairing
    with pytest.raises(PairingError) as e:
        store.remove(a.id)
    assert e.value.reason == "last_browser"
    b = store.pair(store.invite("web")[0]).pairing
    store.remove(a.id)
    assert [x.id for x in store.entries] == [b.id]


def test_removed_token_no_longer_works(store):
    store.pair(store.ensure_bootstrap())
    phone = store.pair(store.invite("app")[0])
    store.remove(phone.pairing.id)
    assert store.check(phone.token) is None


def test_public_view_hides_secrets(store):
    store.pair(store.ensure_bootstrap())
    store.pair(store.invite("esp32")[0])
    for item in store.public():
        assert set(item) == {"id", "role", "name", "created", "last_seen"}


def test_normalize_code():
    assert normalize_code(" ab2c-d3ef ") == "AB2CD3EF"


def test_rename(store):
    a = store.pair(store.ensure_bootstrap(), name="A").pairing
    b = store.pair(store.invite("app")[0], name="B").pairing
    with pytest.raises(PairingError):
        store.rename(b.id, "A")
    assert store.rename(b.id, "Kitchen phone").name == "Kitchen phone"
    assert a.name == "A"

"""Tests for the 0x00-delimited frame reassembly."""

from __future__ import annotations

from transport import FrameExtractor


def test_single_frame_roundtrip():
    ex = FrameExtractor()
    frame = b"\x00abc\x00"
    out = list(ex.push(frame))
    assert out == [frame]


def test_split_across_chunks():
    ex = FrameExtractor()
    assert list(ex.push(b"\x00ab")) == []
    assert list(ex.push(b"cd\x00")) == [b"\x00abcd\x00"]


def test_multiple_frames_in_one_chunk():
    ex = FrameExtractor()
    out = list(ex.push(b"\x00aa\x00\x00bb\x00"))
    assert out == [b"\x00aa\x00", b"\x00bb\x00"]


def test_leading_bytes_form_a_frame_that_decodes_to_garbage():
    # The extractor is content-agnostic: bytes before the first 0x00 are closed
    # into their own frame, which the protocol client later rejects on COBS/CRC.
    ex = FrameExtractor()
    out = list(ex.push(b"junk\x00payload\x00"))
    assert out == [b"\x00junk\x00", b"\x00payload\x00"]


def test_runaway_frame_is_dropped_on_overflow():
    ex = FrameExtractor(max_frame=4)
    # No delimiter within max_frame -> buffer resets, nothing emitted.
    assert list(ex.push(b"abcdefgh")) == []
    # A clean frame after resync is still recovered.
    assert list(ex.push(b"\x00ok\x00")) == [b"\x00ok\x00"]

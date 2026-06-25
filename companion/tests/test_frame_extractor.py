"""Tests for the 0x00-delimited frame reassembly."""

from __future__ import annotations

from hexapod_companion.transport import FrameExtractor


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


def test_leading_garbage_before_first_delimiter_is_dropped():
    ex = FrameExtractor()
    out = list(ex.push(b"junk\x00payload\x00"))
    assert out == [b"\x00payload\x00"]

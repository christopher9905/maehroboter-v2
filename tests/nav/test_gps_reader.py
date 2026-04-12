import time
import pytest
from unittest.mock import MagicMock, patch
from mower.nav.gps_reader import GpsReader, GpsFix, RTK_FIXED, RTK_FLOAT


# A valid GGA sentence with RTK Fixed quality (field 6 = 4)
RTK_FIXED_GGA = (
    "$GNGGA,123519.00,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,,*72\r\n"
)
# RTK Float (field 6 = 5)
RTK_FLOAT_GGA = (
    "$GNGGA,123519.00,4807.038,N,01131.000,E,5,08,1.5,545.4,M,46.9,M,,*7E\r\n"
)
# No fix (field 6 = 0)
NO_FIX_GGA = (
    "$GNGGA,123519.00,4807.038,N,01131.000,E,0,00,99.9,0.0,M,0.0,M,,*75\r\n"
)



class TestGpsFix:
    def test_rtk_fixed_produces_fix(self):
        fixes: list[GpsFix] = []
        reader = GpsReader(port="/dev/fake")
        reader.on_fix = fixes.append
        reader._parse_gga(RTK_FIXED_GGA.strip())
        assert len(fixes) == 1
        fix = fixes[0]
        assert fix.fix_quality == RTK_FIXED
        assert abs(fix.lat - 48.117300) < 0.0001
        assert abs(fix.lon - 11.516667) < 0.0001
        assert fix.utm_x > 0
        assert fix.utm_y > 0

    def test_rtk_float_fix_quality(self):
        fixes: list[GpsFix] = []
        reader = GpsReader(port="/dev/fake")
        reader.on_fix = fixes.append
        reader._parse_gga(RTK_FLOAT_GGA.strip())
        assert fixes[0].fix_quality == RTK_FLOAT

    def test_no_fix_not_published(self):
        fixes: list[GpsFix] = []
        reader = GpsReader(port="/dev/fake")
        reader.on_fix = fixes.append
        reader._parse_gga(NO_FIX_GGA.strip())
        assert len(fixes) == 0

    def test_non_gga_sentence_ignored(self):
        fixes: list[GpsFix] = []
        reader = GpsReader(port="/dev/fake")
        reader.on_fix = fixes.append
        reader._parse_gga("$GNRMC,123519.00,A,4807.038,N,01131.000,E,0.1,0.0,010101,,,A*6E")
        assert len(fixes) == 0

    def test_gga_forwarded_to_ntrip_callback(self):
        forwarded: list[str] = []
        reader = GpsReader(port="/dev/fake")
        reader.on_fix = lambda _: None
        reader.on_gga_sentence = forwarded.append
        reader._parse_gga(RTK_FIXED_GGA.strip())
        assert len(forwarded) == 1
        assert "GNGGA" in forwarded[0]

    def test_utm_zone_populated(self):
        fixes: list[GpsFix] = []
        reader = GpsReader(port="/dev/fake")
        reader.on_fix = fixes.append
        reader._parse_gga(RTK_FIXED_GGA.strip())
        assert fixes[0].utm_zone_number == 32  # Munich is UTM zone 32U
        assert fixes[0].utm_zone_letter == "U"

    def test_malformed_sentence_does_not_raise(self):
        fixes: list[GpsFix] = []
        reader = GpsReader(port="/dev/fake")
        reader.on_fix = fixes.append
        reader._parse_gga("$GNGGA,BAD_DATA,,,,,,,,,,,,,")  # should not raise
        assert len(fixes) == 0

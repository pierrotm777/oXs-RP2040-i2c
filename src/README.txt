oXs - common FVP/FVN LiPo cell simulation for CRSF + SRXL2

This package is based on the latest crsf_out.cpp supplied by the user.

Files:
- crsf_out.cpp  : latest user file, minimally corrected
- tools.cpp     : generic 8S FVP/FVN source
- tools.h       : generic getForcedCellValues() declaration
- srxl2.cpp     : SRXL2 uses generic cells for Spektrum 0x3A
- param.cpp     : no cell simulation hidden in ADS_2_4
- CRSF_DIFF.patch

CRSF changes:
- removed extern crsfSimCellsPosMv / crsfSimCellsNegMv
- fillFrameCells() now calls getForcedCellValues(forcedFields)
- no CRSF telemetry frame format or cell values changed

Generic helper accepts both:
- forcedFields = 1 / 2
- legacy forcedFields = 'P' / 'N'

FVP cells:
4.20 4.18 4.16 4.14 4.12 4.10 4.08 4.06 V

FVN cells:
3.20 3.18 3.16 3.14 3.12 3.10 3.08 3.06 V

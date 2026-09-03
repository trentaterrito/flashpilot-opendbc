# Upstream MADS reference core — offline only

The two headers are exact imports from the commit in UPSTREAM.json. Only
`tests/sunnypilot_mads_harness.c` includes them in this branch. No production
initialization, safety hook, feature toggle, steering permission, or host caller
is connected. Characterization tests preserve upstream behavior, including
policies that still require review before use in FlashPilot.

This software is licensed under a custom license requiring permission for use.
This project uses software from Haibin Wen and SUNNYPILOT LLC and is licensed
under a custom license requiring permission for use. See LICENSE.md; commercial,
for-profit, or closed-source use requires written permission from the authors.

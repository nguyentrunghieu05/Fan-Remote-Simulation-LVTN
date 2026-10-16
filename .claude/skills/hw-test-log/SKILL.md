---
name: hw-test-log
description: Run a test on the real NUCLEO-G0B1RE board, show it live in MobaXterm, screenshot the log and insert the figures into the thesis Word file without touching its formatting. Use when asked to test firmware on the board, capture MobaXterm logs, or add board test results to the report.
---

# Hardware test, MobaXterm log, Word figure

Scripts are in `scripts/` next to this file. They need Python with `pyserial`, `pillow` and `pypdf`,
STM32CubeProgrammer (`STM32_Programmer_CLI` on PATH) and MobaXterm.

## 1. Prepare

- Ask the user which COM port the ST-LINK VCP uses (it was COM17) and that the board is connected.
- MobaXterm must be open with a **local terminal** tab active and **no serial session on that port**
  (only one program can open the port). The window is maximized by the scripts.
  `moba.TERM_BOX` is the terminal area for a 1920x1080 screen at 100 %; check the first screenshot
  and adjust it if the screen is different.
- Build and flash first if needed: `make BOOT=1`, flash with `STM32_Programmer_CLI -c port=SWD -w <file> -v -rst`.

## 2. Run the test and take the screenshots

Write the test as a list of steps for `session.py` and start it inside the MobaXterm tab, so the
log appears live there:

```
python scripts/moba.py type "python <abs path>/scripts/session.py COM17 reset until:Type:15 send:fw shot:<out>/H43_boot.png"
```

Steps: `send:<cmd>`, `wait:<s>`, `until:<text>[:<s>]`, `reset` (hard reset via ST-LINK),
`upload:<bin>:<version>` (runs `tools/fwupdate.py --no-install`), `shot:<png>`, `hold:<s>`.
The raw output is also appended to `%TEMP%/hw_session.log`.

- One screenshot per figure, only the lines that support the text.
- Read every PNG back before using it, and quote numbers in the report exactly from the log.
- A hard reset through ST-LINK is **not** a power cut. Say "reset cứng qua ST-LINK"; say
  "rút nguồn" only if the cable was really unplugged.

## 3. Insert into Word (text only, never reformat)

The user formats the thesis by hand. Never open it in Word to edit, never run TOC `Update()`.

1. Check there is no `~$` lock file next to the .docx (if there is, ask the user to close Word).
   Copy the .docx to the scratchpad as a backup.
2. Write a `spec.json` (format in the header of `scripts/docx_insert.py`): where to insert,
   which existing heading / paragraph / figure / list-of-figures entry to copy, the new items,
   figures to renumber after the insertion, other one-to-one text replacements.
3. Work on a copy in the scratchpad:
   `python scripts/docx_insert.py insert IN.docx draft.docx spec.json <img dir>`
4. Export the draft to PDF with a timeout (never call Word without it):
   `powershell -File scripts/word_pdf.ps1 <abs draft.docx> <abs check.pdf> 180`
5. Fix TOC and list-of-figures page numbers from that PDF
   (OFFSET = PDF page index from 0 minus printed page number, 10 for the current thesis):
   `python scripts/docx_insert.py pages draft.docx draft2.docx check.pdf 10`
6. Check before overwriting the real file: every zip part except `word/document.xml`,
   `word/_rels/document.xml.rels` and the new `word/media/log_*` is unchanged, figure numbers
   run 1..N with no gap, and only the intended paragraphs differ from the backup.
7. Show the user the list of changed paragraphs and ask before copying the draft over the real file.

## 4. Slides and script

Only after the Word text is final: change the slides and the presentation script to match it,
nothing beyond what the report says.

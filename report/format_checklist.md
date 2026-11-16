# Report Format Checklist

## Purpose
This checklist must be verified against the actual exported PDF before submission.
Do not mark an item as ✅ until it has been physically inspected in the PDF.

---

## Page Setup

| Item | Requirement | Status |
|------|-------------|--------|
| Paper size | A4 portrait | ☐ |
| Left margin | 3.0 cm | ☐ |
| Right margin | 2.5 cm | ☐ |
| Top margin | 2.5 cm | ☐ |
| Bottom margin | 2.5 cm | ☐ |
| Single-line page border | Every page including front matter | ☐ |
| Single-sided printing | All pages print on one side only | ☐ |

---

## Typography

| Item | Requirement | Status |
|------|-------------|--------|
| Body font | Times New Roman | ☐ |
| Body font size | 12 pt | ☐ |
| Body line spacing | 1.5 (not 2.0, not 1.0) | ☐ |
| Body alignment | Justified (not left-aligned) | ☐ |
| Section headings | 14 pt bold | ☐ |
| Subsection headings | 12 pt bold (or 13 pt; check sample report) | ☐ |
| Captions | Below each figure; above each table | ☐ |
| Code blocks | Monospace font (Courier New 10pt acceptable) | ☐ |

---

## Page Numbering

| Item | Requirement | Status |
|------|-------------|--------|
| Title page | No page number | ☐ |
| Front matter (Certificate through List of Tables) | Roman numerals (i, ii, iii...) centred at bottom | ☐ |
| Body (Chapter 1 onwards) | Arabic numerals (1, 2, 3...) centred at bottom | ☐ |
| Body starts at page 1 | Chapter 1 = page 1 | ☐ |
| References section | Continues Arabic numbering | ☐ |
| Appendices | Continues Arabic numbering | ☐ |

---

## Front Matter

| Item | Requirement | Status |
|------|-------------|--------|
| Title page | Title, group, course, date, institution, supervisor | ☐ |
| Certificate | Institution/department certificate template | ☐ |
| Declaration | Signed by all group members | ☐ |
| Acknowledgement | Thanks to supervisor [FACULTY_NAME] | ☐ |
| Abstract | 150–200 words, single paragraph | ☐ |
| Table of Contents | Complete, with correct page numbers | ☐ |
| List of Figures | All figures numbered and captioned | ☐ |
| List of Tables | All tables numbered and captioned | ☐ |

---

## Body Content

| Item | Requirement | Status |
|------|-------------|--------|
| Total body pages | 25–35 pages (Chapters 1–12 + References) | ☐ |
| All section headings numbered | 1.1, 1.2, 2.1 etc. | ☐ |
| Novelty section present | Chapter 9 | ☐ |
| Contribution section present | Chapter 9 (or separate) | ☐ |
| References ≥ 8 | All in IEEE format | ☐ |
| Every reference cited in text | No uncited references | ☐ |
| Every in-text citation in list | No dangling citations | ☐ |
| All figures have captions | "Figure X: Description" below figure | ☐ |
| All tables have captions | "Table X: Description" above table | ☐ |
| All figures/tables in List of Figures/Tables | Matching page numbers | ☐ |

---

## Figures and Tables

| Item | Requirement | Status |
|------|-------------|--------|
| All plots at ≥ 300 DPI | Check PNG resolution | ☐ |
| Axes labelled with units | "Time (s)", "Tilt (degrees)" | ☐ |
| Legends on all multi-line plots | "PID", "LQR" with consistent colours | ☐ |
| No pixelated/blurry figures | View at 100% in PDF | ☐ |
| Data in tables matches CSV | Spot-check 3 numbers against results/ CSVs | ☐ |

---

## Mandatory Content

| Item | Requirement | Status |
|------|-------------|--------|
| Index (Table of Contents) | ✓ present | ☐ |
| List of Figures | ✓ present | ☐ |
| List of Tables | ✓ present | ☐ |
| Novelty section | ✓ present in body | ☐ |
| Contribution section | ✓ present | ☐ |

---

## Final Checks

| Item | Requirement | Status |
|------|-------------|--------|
| PDF exported and inspected | First and last page viewed | ☐ |
| Page borders on all pages | Scroll through entire PDF | ☐ |
| No blank pages (unless intentional) | Check between chapters | ☐ |
| Printed copy bound | Required for submission | ☐ |
| Soft copy submitted (if required) | PDF via portal/email | ☐ |
| Repository URL in report | GitHub link in Chapter 1 or Appendix | ☐ |
| Final commit hash stated | "Results generated at commit XXXXXXX" | ☐ |
| Placeholders removed | Search for "[" in final PDF — none should exist | ☐ |

---

## Formatting Tool Guidance

### Using python-docx
If generating a .docx with python-docx:
1. Set page size: `section.page_width = Mm(210)`, `section.page_height = Mm(297)`
2. Set margins: `section.left_margin = Mm(30)`, `section.right_margin = Mm(25)`, etc.
3. Add page border: Use XML manipulation (python-docx does not natively support borders; use `oxml` library or a template .docx with borders already set)
4. Add header/footer with page numbers: Use `header_footer.add_paragraph()`
5. Run styles: Create "Heading 1" style with 14pt bold Times New Roman

### Using LaTeX
If using LaTeX:
```latex
\usepackage[a4paper, left=3cm, right=2.5cm, top=2.5cm, bottom=2.5cm]{geometry}
\usepackage{fancyhdr}
\usepackage[framemethod=TikZ]{mdframed}  % for page border
\setmainfont{Times New Roman}
\linespread{1.5}
\setlength{\parskip}{0pt}
```

### Inspection command
To inspect PDF margins and layout:
```bash
pdfinfo report.pdf  # check page size
pdfimages -list report.pdf | head -20  # check image DPI
```

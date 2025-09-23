# Website Contribution Guide (MkDocs Material)

Welcome! This page shows **everything you need** to add or edit pages on our docs site — no prior experience required.

---

## 0) TL;DR (10-minute checklist)

1. Install: `pip install mkdocs mkdocs-material`  
2. Clone repo → open the folder.  
3. Run locally: `mkdocs serve` → open http://127.0.0.1:8000  
4. Create your page under `docs/...` (e.g., `docs/tutorials/my_page.md`).  
5. Add it to the sidebar in `mkdocs.yml` under `nav:`.  
6. Write content (use examples below for **links, images, videos, math, code**).  
7. Save → preview refreshes automatically.  
8. Commit & push → site auto-deploys.

---

## 1) Install & run locally

~~~bash
# 1) Install once
pip install mkdocs mkdocs-material

# 2) In the repo root:
mkdocs serve
# Open the link shown in the terminal (usually http://127.0.0.1:8000)
~~~

> If your page doesn’t show in the sidebar, you probably forgot to add it to `mkdocs.yml` (see section 5).

---

## 2) Repo structure (what goes where)

.

├─ mkdocs.yml # site config & sidebar (nav)

└─ docs/ # all content lives here

├─ index.md # homepage

├─ media/ # images & videos you upload

├─ tutorials/ # tutorial pages

├─ departments/ # team/tech sections

├─ overrides/ # optional CSS tweaks

└─ javascripts/ # optional JS (e.g., MathJax helper)


**Rules**
- Put **Markdown pages** inside `docs/`.
- Put **images/videos** in `docs/media/`.
- Use **relative paths** in links (don’t start with `/`, don’t include `docs/`).

---

## 3) One-time config (math, captions, responsive iframes)

Add these to `mkdocs.yml` if not already:

~~~yaml
theme:
  name: material

markdown_extensions:
  - attr_list        # width=..., target=..., etc.
  - md_in_html       # lets Markdown work inside <figure>…</figure>
  - admonition
  - pymdownx.details
  - pymdownx.superfences
  - pymdownx.tabbed:
      alternate_style: true
  - pymdownx.tasklist:
      custom_checkbox: true
  - pymdownx.arithmatex:
      generic: true

extra_javascript:
  - javascripts/mathjax.js
  - https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js

extra_css:
  - overrides/styles.css
~~~

Create `docs/javascripts/mathjax.js`:

~~~js
window.MathJax = {
  tex: { inlineMath: [['$', '$'], ['\\(', '\\)']] },
  options: { skipHtmlTags: ['script','noscript','style','textarea','pre'] }
};
~~~

Create `docs/overrides/styles.css`:

~~~css
/* 16:9 responsive iframe wrapper for YouTube/Vimeo */
.video { position: relative; padding-top: 56.25%; height: 0; overflow: hidden; margin: 1rem 0; }
.video iframe { position: absolute; inset: 0; width: 100%; height: 100%; border: 0; }

/* Optional: figure + caption styling */
figure { text-align: center; margin: 1rem 0; }
figcaption { margin-top:.4rem; font-size:.9rem; color: var(--md-default-fg-color--light); }
~~~

---

## 4) Create a page

Create `docs/tutorials/webpage_setup.md` (for example). Start with:

~~~md
# Page Title

A few sentences describing what this page covers.

## Section heading

Content goes here.
~~~

---

## 5) Add your page to the sidebar (navigation)

Open `mkdocs.yml` and edit the `nav:` section. Use paths under `docs/`.

~~~yaml
nav:
  - Home: index.md
  - Tutorials:
      - LiDAR Setup: tutorials/lidar-setup.md
      - Webpage Setup: tutorials/webpage_setup.md   # ← your new page
~~~

> **Case-sensitive!** `perception` ≠ `Perception`.  
> If you forget this step, your page exists but won’t appear in the sidebar.

---

## 6) Write content: the essentials

### A) Links

- Link to **another page** (path is relative to *this* file):
  ~~~md
  [Home](../../index.md)
  [LiDAR Setup](../../tutorials/lidar-setup.md)
  [Perception overview](../perception/overview.md)
  ~~~

- Link to a **section** on another page (anchors are kebab-case):
  ~~~md
  [Calibration steps](../../tutorials/lidar-setup.md#calibration-steps)
  ~~~
  Or set your own id:
  ~~~md
  ## Calibration steps {#calib}
  [Jump there](../../tutorials/lidar-setup.md#calib)
  ~~~

- Open in **new tab**:
  ~~~md
  [External site](https://eufs.eu){ target=_blank rel=noopener }
  ~~~

- Inside shown links
  ~~~md
  <link>
  ~~~

### B) Images

Put images in `docs/media/` and use **relative** paths:

~~~md
![Cone map](../../media/wlco.png){ width=600 loading=lazy }
~~~

Clickable thumbnail → full image:

~~~md
[![Cone map](../../media/wlco.png){ width=320 }](../../media/wlco.png){ target=_blank }
~~~

**Captioned figure** (recommended):

~~~html
<figure markdown>
  ![Cone map](../../media/wlco.png){ width=600 loading=lazy }
  <figcaption>Figure 1: Cone clustering output from the VLP-16.</figcaption>
</figure>
~~~

### C) Videos

**YouTube (privacy mode + responsive)**

~~~html
<div class="video">
  <iframe
    src="https://www.youtube-nocookie.com/embed/VIDEO_ID?rel=0"
    title="Demo"
    loading="lazy"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen></iframe>
</div>
~~~

**Online MP4 (direct URL or local file in `docs/media/`)**

~~~html
<video controls preload="metadata" width="100%" playsinline poster="../../media/thumbnail.jpg">
  <source src="../../media/demo.mp4" type="video/mp4">
  Your browser doesn’t support HTML5 video.
</video>
~~~

Autoplay (mobile-friendly):
~~~html
<video controls autoplay muted loop playsinline width="100%">
  <source src="https://example.com/video.mp4" type="video/mp4">
</video>
~~~

> If you see “CORS” errors with remote MP4s, download the file into `docs/media/` and link to it locally.

### D) Math (LaTeX)

Inline: `$\theta^* = \arg\max_\theta p(\mathcal{D}\mid\theta)$` → $\theta^* = \arg\max_\theta p(\mathcal{D}\mid\theta)$

Block:
~~~latex
$$
a = k_p \, e \;+\; k_i \sum (e\,\Delta t)
$$
~~~

### E) Code blocks

Fenced code:
````md
```python
def f(x):
    return x*x

Tabs (multi-language):
~~~md
=== "Python"
```python
print("hello")
```
=== "C++"
```cpp
std::cout << "hello\n";
```
~~~

Admonitions:
~~~md
!!! note
    This is a helpful callout.
~~~

---

## 7) Style & formatting tips

- **Headings**: Start with a single `# Title` at top, then `##`, `###`, …  
- **Images**: Prefer `PNG/JPG` for photos, `SVG/PNG` for diagrams. Use alt text.  
- **Links**: Use **relative** paths, never start with `/`, and don’t include `docs/`.  
- **Filenames**: `lowercase-with-dashes.md` is easiest to link.  
- **Sections**: Use clear headings; the right-hand “Table of contents” will follow as you scroll.  
- **Search**: Full-text (except very common “stop words” like “we”, “the”, …).

---

## 8) Git workflow (basic)

~~~bash
# Create a new branch (recommended)
git checkout -b docs/my-new-page

# Add your file(s)
git add docs/tutorials/webpage_setup.md mkdocs.yml

# Commit
git commit -m "docs: add Webpage Setup tutorial"

# Push
git push -u origin docs/my-new-page
~~~

Open a **Pull Request** on GitHub. After merge to `master` (or the main branch), the site rebuilds automatically via GitHub Actions.

---

## 9) Troubleshooting

| Problem | Likely cause | Fix |
|---|---|---|
| Page doesn’t show in sidebar | Not in `mkdocs.yml` `nav:` | Add the file path under `nav` (case-sensitive) |
| Image not loading | Wrong **relative path** or uppercase/lowercase mismatch | Check path from the current file → `../../media/your.png` |
| Video won’t play (remote MP4) | CORS blocked | Download to `docs/media/` or host on a CDN that allows cross-origin |
| Math not rendering | Missing config | Ensure `pymdownx.arithmatex` + MathJax are in `mkdocs.yml` |
| ToC headings in left sidebar | `toc.integrate` enabled | Remove `toc.integrate` from theme features |
| All dropdowns open by default | `navigation.expand` enabled | Remove `navigation.expand` from theme features |

---

## 10) Quick reference: relative paths

From page… | To `docs/media/wlco.png`
:--|:--
`docs/index.md` | `media/wlco.png`
`docs/tutorials/lidar-setup.md` | `../media/wlco.png`
`docs/departments/perception/theory.md` | `../../media/wlco.png`

---

## 11) Example page template (copy into a new file)

~~~md
# My New Page

Brief description of what this page covers.

## 1. Overview

Here is an inline equation $\theta^* = \arg\max_\theta p(\mathcal{D}\mid\theta)$.

```latex
$$
a = k_p\,e + k_i \sum (e\,\Delta t)
$$
```

## 2. Image with caption

<figure markdown>
  ![Pipeline diagram](../tutorials/media/diagram.png){ width=600 loading=lazy }
  <figcaption>Figure 1: Oh no.</figcaption>
</figure>

## 3. Video

<div class="video">
  <iframe
    src="https://www.youtube.com/watch?v=xvFZjo5PgG0"
    title="Demo"
    loading="lazy"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen></iframe>
</div>

## 4. Useful links

<!-- - [LiDAR Setup](../tutorials/lidar-setup.md#calibration-steps) -->
- [Back to Home](../index.md)

## 5. Code

```python
def normalize(v):
    return v / (abs(v) + 1e-8)
```
~~~

---

## 12) Before you push

- Run locally: `mkdocs serve`  
- Fix warnings (missing pages in `nav`, bad links)  
- Check images/videos load  
- Commit & push
```
::contentReference[oaicite:0]{index=0}


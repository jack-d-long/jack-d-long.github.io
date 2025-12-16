# AGENT PLAYBOOK

Guidelines for agents working in `jack-d-long.github.io`, a Zola-powered student portfolio hosted on GitHub Pages.

## 1. Understand the Stack
- **Generator**: Zola. Edit Markdown in `content/`, Sass in `sass/`, templates in `templates/`. Avoid touching `public/`; it holds generated assets.
- **Build**: Use `zola serve` for local preview and `zola build` to regenerate `public/`. Keep `config.toml` aligned with GitHub Pages (`base_url = "https://jack-d-long.github.io/"`).
- **Assets**: Custom fonts live in `sass/fonts.scss`; color themes are under `sass/theme/`. Images and static files belong in `static/`.

## 2. Coding Standards
- Prefer semantic HTML in templates and lightweight, well-commented Sass. Respect existing CSS variables and media queries.
- When editing Sass, run `zola build` to ensure `public/` stays in sync or explain if the build step was skipped.
- Keep dependencies minimal; network installs may be restricted.

## 3. Content & Tone (Student Portfolio)
- Preserve a professional-but-approachable tone that highlights academic projects, research, and coursework outcomes.
- Protect privacy: omit personal contact info unless already public, and avoid sharing classmates' data.
- Showcase learning reflections and measurable impact; link to GitHub repos where available.
- Ensure accessibility (meaningful headings, alt text) and mobile readability (check breakpoints in `sass/main.scss`).

## 4. Operational Checklist
1. **Plan before editing**: outline affected files/sections.
2. **Modify source files only** (`content/`, `sass/`, `templates/`, `static/`).
3. **Validate**: run `zola build` and, if feasible, `zola serve` for spot checks.
4. **Document changes**: summarize affected files and mention any skipped verification steps.
5. **Deploy considerations**: GitHub Pages publishes from `public/` or GitHub Actions. Coordinate with the maintainer before force-pushing artifacts.

## 5. Communication
- Report blockers early (e.g., network restrictions, missing toolchains).
- Use concise summaries referencing file paths and line numbers.
- Suggest next steps when appropriate (e.g., content review, image optimization).

Following this playbook keeps the student portfolio consistent, professional, and deployment-ready.

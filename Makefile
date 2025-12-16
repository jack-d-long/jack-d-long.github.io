# Paths
PDF_DIR := content/portfolio/4300
PDF_SRC := $(PDF_DIR)/report.tex
PDF_OUT := $(PDF_DIR)/report.pdf
STATIC_PDF := static/4300/report.pdf

.PHONY: pdf build clean

# Rebuild the LaTeX PDF (with bibliography) and copy it to static for embedding.
pdf: $(PDF_OUT)
	@mkdir -p $(dir $(STATIC_PDF))
	cp $(PDF_OUT) $(STATIC_PDF)

$(PDF_OUT): $(PDF_SRC) $(PDF_DIR)/index.tex $(PDF_DIR)/refs.bib
	cd $(PDF_DIR) && pdflatex -interaction=nonstopmode -halt-on-error report.tex
	cd $(PDF_DIR) && biber report
	cd $(PDF_DIR) && pdflatex -interaction=nonstopmode -halt-on-error report.tex
	cd $(PDF_DIR) && pdflatex -interaction=nonstopmode -halt-on-error report.tex

# Full site build: rebuild LaTeX then run zola.
build: pdf
	zola build

clean:
	cd $(PDF_DIR) && rm -f report.{aux,bbl,bcf,blg,log,out,run.xml,toc}

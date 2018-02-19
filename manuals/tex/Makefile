TEXFILES = $(wildcard *.tex)
BUILD_DIR = build
LATEX_OPS = -output-directory $(BUILD_DIR)
BIBTEX = export BIBINPUTS=../ && cd $(BUILD_DIR) && bibtex
#BASE_DIR := $(shell echo `pwd` | sed -e 's/\/shed\/papers.*/\/shed\/papers\/tools/g')

all:	
	make -C ../

figs:
	$(MAKE) -C ../figs

images:
	$(MAKE) -C ../images

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.pdf: %.tex %.toc $(TEXFILES) figs images | $(BUILD_DIR)
	$(PDFLATEX) $(LATEX_OPS) $*.tex 
	$(BIBTEX) $*
	$(PDFLATEX) $(LATEX_OPS) $*.tex
	$(PDFLATEX) $(LATEX_OPS) $*.tex
	python $(TOOLS_DIR)/cat_tex.py $* $(BUILD_DIR)/$*

$(BUILD_DIR)/%.ps: %.tex %.toc $(TEXFILES) figs images | $(BUILD_DIR)
	$(LATEX) $(LATEX_OPS) $*.tex 
	$(BIBTEX) $*
	$(LATEX) $(LATEX_OPS) $*.tex
	$(LATEX) $(LATEX_OPS) $*.tex
	$(PSLATEX) $(BUILD_DIR)/$*.dvi -o $(BUILD_DIR)/$*.ps

%.blg: %.bib | $(BUILD_DIR)
	$(PDFLATEX) $(LATEX_OPS) $*.tex
	$(BIBTEX) $*
	$(PDFLATEX) $(LATEX_OPS) $*.tex

%.toc: %.tex | $(BUILD_DIR)
	$(PDFLATEX) $(LATEX_OPS) $*.tex

clean:
	/bin/rm -f $(MAINFILE).pdf *.dvi *.aux *.ps *~ *.log *.out *.ent
	/bin/rm -f *.lot *.lof *.toc *.blg *.bbl url.sty
	/bin/rm -rf $(BUILD_DIR)

view:
	$(MAKE) -C ../ view

publish: $(MAINFILE).pdf
	scp $(MAINFILE).pdf prabal@calumet.eecs.umich.edu:/net/www/z/p/prabal/public_html/pubs/papers/$(MAINFILE).pdf

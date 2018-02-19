MAINFILE = paper
TEXFILES = $(wildcard tex/*.tex)
TOOLS_DIR = $(SHED_REPO)/papers/tools
UNAME := $(shell uname)

ifeq ($(UNAME), Linux)
	VIEW=evince
	LATEX=latex
	PDFLATEX=pdflatex
	PSLATEX=dvips
endif
ifeq ($(UNAME), Darwin)
	VIEW=open
	LATEX=latex
	PDFLATEX=pdflatex
	PSLATEX=dvips
endif
ifeq ($(VIEW),)
$(error Unknown operating system, please update with a UNAME entry and appropriate config)
endif

ifeq ($(SHED_REPO),)
$(error You need to set the environment variable "SHED_REPO" to root of shed directory)
endif

export

.SECONDARY:

# these two let you remake if you change a tex file
# probably should be merged into one target somehow
tex/%.pdf:	$(TEXFILES)
	$(MAKE) -C ./tex $*.pdf

tex/%.ps:	$(TEXFILES)
	$(MAKE) -C ./tex $*.ps

$(MAINFILE).%:	tex/build/$(MAINFILE).%
	if [ ! -e $@ ]; then ln -s $<; fi

all:	$(MAINFILE).pdf

clean:
	$(MAKE) -C ./images clean
	$(MAKE) -C ./figs clean
	$(MAKE) -C ./tex clean
	/bin/rm -f $(MAINFILE).pdf
	/bin/rm -f $(MAINFILE).ps

pn: $(MAINFILE).pdf
	mv tex/$(MAINFILE).tex tex/$(MAINFILE).tex.backup
	sed 's/%\\pagenumbering{arabic}/\\pagenumbering{arabic}/' tex/$(MAINFILE).tex.backup > tex/$(MAINFILE).tex
	$(MAKE) -C ./tex
	mv tex/$(MAINFILE).tex.backup tex/$(MAINFILE).tex
	mv tex/build/$(MAINFILE).pdf tex/build/$(MAINFILE)_pagenumbers.pdf
	mv tex/build/$(MAINFILE).ps tex/build/$(MAINFILE)_pagenumbers.ps
	$(MAKE) -C ./tex

spell:	$(MAINFILE).ps
	@echo -e "----------------\n  SPELL CHECK\n----------------"
	@$(TOOLS_DIR)/spell_check.sh $(PWD)/tex/build/$(MAINFILE).dvi $(PWD)/tex/ignore.txt "$(TEXFILES)"

style:	$(TEXFILES)
	@echo -e "----------------\n  SYNTAX CHECK\n----------------"
	@$(TOOLS_DIR)/style-check/style-check.rb -v -r $(TOOLS_DIR)/style-check/rules tex/*.tex

check:	spell style

view: $(MAINFILE).pdf
	$(VIEW) $(MAINFILE).pdf

publish: $(MAINFILE).pdf
	scp $(MAINFILE).pdf prabal@calumet.eecs.umich.edu:/net/www/z/p/prabal/public_html/pubs/papers/$(MAINFILE).pdf

.PHONY: all clean view publish check spell style

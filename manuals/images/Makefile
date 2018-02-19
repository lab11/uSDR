BUILD_DIR = build

SVGS := $(wildcard *.svg)
EPSS := $(wildcard *.eps)
PSS  := $(wildcard *.ps)
PDFS := $(wildcard *.pdf)
JPGS := $(wildcard *.jpg)
PNGS := $(wildcard *.png)
GIFS := $(wildcard *.gif)

PICS := $(JPGS) $(PDFS) $(PSS) $(PNGS) $(EPSS) $(GIFS) $(SVGS)
PICS := $(patsubst %,$(BUILD_DIR)/%,$(PICS))

all: $(PICS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

.SECONDEXPANSION:

$(BUILD_DIR)/%.svg: $$*.svg | $(BUILD_DIR)
	ln -f $*.svg $@
	rsvg-convert -f ps -o $(BUILD_DIR)/$*.ps $@
	ps2eps $(BUILD_DIR)/$*.ps
	epstopdf $(BUILD_DIR)/$*.eps

$(BUILD_DIR)/%.pdf: $$*.pdf | $(BUILD_DIR)
	ln -f $*.pdf $(BUILD_DIR)/$*.pdf
	pdftops -eps $(BUILD_DIR)/$*.pdf $(BUILD_DIR)/$*.eps

$(BUILD_DIR)/%.eps: $$*.eps | $(BUILD_DIR)
	ln -f $*.eps $@
	epstopdf $(BUILD_DIR)/$*.eps

$(BUILD_DIR)/%: $$* | $(BUILD_DIR)
	ln -f $* $@
	convert $@ $(basename $@).eps
	epstopdf $(basename $@).eps

clean:
	rm -rf build

.PHONY: all clean

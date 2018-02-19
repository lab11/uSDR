ROOT_DIRECTORY=.
DIRS := ${shell find ${ROOT_DIRECTORY} -mindepth 1 -maxdepth 1 -type d -print | grep -v svn}

PLTS := $(shell find ${ROOT_DIRECTORY} -name *.plt)
PDFS := $(patsubst %.plt, %.pdf, $(PLTS))


A2PING_OPTS = --gsextra "-dUseCIEColor -dPDFSETTINGS=/printer -dSubsetFonts=true -dAutoRotatePages=/None -dEmbedAllFonts=true"


all: $(DIRS) $(PDFS)

$(DIRS):
	if [ -e $@/Makefile ]; then $(MAKE) -C $@; fi

%.eps: %.plt
	cd `dirname $<`; \
	gnuplot `basename $<`;

%.pdf: %.eps
	cd `dirname $<`; \
	a2ping $(A2PING_OPTS) `basename $<` -o `basename $@`

clean:
	for dir in $(DIRS) ; do \
		cd $$dir; \
		if [ -e Makefile ]; then $(MAKE) clean; fi; \
		rm -f ./*.eps; \
		rm -f ./*.pdf; \
		cd ..; \
	done

.PHONY: $(DIRS)
.PRECIOUS: %.eps
	

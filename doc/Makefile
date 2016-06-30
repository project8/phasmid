DOC=r2daq-reference
SOURCE=$(DOC).tex
BIBS=$(DOC).bib
APPEND=ROACH2-ASSYv4.pdf
IMAGES=roach2-front.png roach2-rear.png
LATEX=pdflatex
BIBTEX=bibtex
GARBAGE=*.aux *.bbl *.blg *.lof *.log *.lot *.out *.toc

.PHONY : default

defalut : clean all

.PHONY : all

all : $(DOC).pdf

.PHONY : clean

clean:
	rm -f $(GARBAGE)
	rm -f $(DOC).pdf

$(DOC).aux : $(SOURCE) $(BIBS) $(IMAGES) $(APPEND)
	$(LATEX) $(SOURCE)


$(DOC).bbl : $(DOC).aux
	$(BIBTEX) $(DOC).aux

$(DOC).pdf : $(DOC).bbl
	$(LATEX) $(SOURCE)
	$(LATEX) $(SOURCE)

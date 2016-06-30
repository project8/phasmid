DOC=r2daq-reference
SOURCE=$(DOC).tex
BIBS=$(DOC).bib
APPEND=ROACH2-ASSYv4.pdf
IMAGES=roach2-front.png roach2-rear.png
LATEX=pdflatex
BIBTEX=bibtex
GARBAGE=*.aux *.bbl *.blg *.lof *.log *.out *.toc

$(DOC).aux : $(SOURCE) $(BIBS) $(IMAGES) $(APPEND)
	$(LATEX) $(SOURCE)


$(DOC).bbl : $(DOC).aux
	$(BIBTEX) $(DOC).aux

$(DOC).pdf : $(DOC).bbl
	$(LATEX) $(SOURCE)
	$(LATEX) $(SOURCE)

all : $(DOC).pdf

clean:
	rm -f $(GARBAGE)
	rm -f $(DOC).pdf

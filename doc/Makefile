# Remove intermediate files if there is an error
.DELETE_ON_ERROR:

%.wiki : %.tex
	pandoc -f latex -t mediawiki $< -o $@

# http://stackoverflow.com/questions/12343428/latex-reference-and-makefile
%.pdf %.aux %.idx: %.tex
	pdflatex -file-line-error -interaction=batchmode $< 
	pdflatex -file-line-error -interaction=batchmode $< 

all: opencl_readme.pdf opencl_readme.wiki

clean:
	rm -f *.aux *.log *.toc *.out

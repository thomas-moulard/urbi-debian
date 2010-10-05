# Rules to convert several common figure formats to formats well
# supported by pdflatex (png, jpg, pdf).

CONVERT ?= convert
DIA ?= dia
DOT ?= dot
ENSURE_TARGET_DIR ?= mkdir -p $(dir $@) || true
EPSTOPDF ?= epstopdf
FDP ?= fdp
FIG2DEV ?= fig2dev
GNUPLOT ?= gnuplot
PDFCROP ?= pdfcrop

CONVERT_TO_PDF =				\
  { $(ENSURE_TARGET_DIR) ; } &&			\
  $(CONVERT) $< pdf:$@.tmp &&			\
  mv $@.tmp $@

# Formats to convert to PDF.
SHARE_EXTS_TO_PDF = \
  dot fig fdp gif id tif tiff pbm pgm ppm pdffig plt $(EXTS_TO_PDF)
# Formats to convert to PNG.
SHARE_EXTS_TO_PNG = dia $(EXTS_TO_PNG)

# FILES
# convert_ext SRC-EXT, DST-EXT, FILEs
# -----------------------------------
# Return the sorted list of $(FILES) with SRC-EXT as extension,
# changing to DST-EXT.
share_convert_ext =					\
   $(sort						\
          $(patsubst %.$(1),%.$(2),			\
                     $(filter %.$(1),$(3))))


# FILES
# convert_exts SRC-EXTS, DST-EXT, FILES
# -------------------------------------
# Map all the extensions in SRC-EXTS to DST-EXT of the $(FILES) list.
share_convert_exts =					\
  $(foreach ext,					\
            $(1),					\
            $(call share_convert_ext,$(ext),$(2),$(3)))


# FILES
# share_convert_to_pdf FILES
# --------------------------
# Choose the most appropriate format for PDFLaTeX for the FILES.
# Beware that unknown formats are left out.
share_convert_to_pdf =						\
    $(call share_convert_exts,$(SHARE_EXTS_TO_PDF),pdf,$(1))	\
    $(call share_convert_exts,$(SHARE_EXTS_TO_PNG),png,$(1))


# There seems to be more bugs in dia -> fig -> pdf than dia -> png.
%.png: %.dia
	$(ENSURE_TARGET_DIR)
	$(DIA) -n -e $@.tmp $<
	mv $@.tmp $@

%.png: %.fdp
	$(ENSURE_TARGET_DIR)
	$(FDP) -Tpng $< -o $@.tmp
	mv $@.tmp $@


## This does not work properly, especially when the output is bigger
## than A4, in which case ps2epsi crops.
##
## %.eps: %.dot
## 	dot -Gpage=595.842 -Tps2 $< -o $*.ps
## # This line: [ /CropBox [36 36 97 89] /PAGES pdfmark
## # breaks the ps2pdf output.
## 	sed -i '/CropBox/d' $*.ps
## 	ps2epsi $*.ps $@

%.fig: %.dot
	$(ENSURE_TARGET_DIR)
	$(DOT) -Tfig $< -o $@.tmp
	mv $@.tmp $@

%.pdf: %.dot
	$(ENSURE_TARGET_DIR)
# Some versions of dot don't support PDF output.
	if $(DOT) -Tpdf </dev/null 2>/dev/null; then	\
	  $(DOT) -Tpdf $< -o $@.t1 &&			\
	  $(PDFCROP) $@.t1 $@.tmp;			\
	else						\
	  $(DOT) -Tfig $< -o $@.t1 &&			\
	  $(FIG2DEV) -Lpdf -p dummy $@.t1 $@.tmp;	\
	fi
	rm $@.t1
	mv $@.tmp $@

%.fig: %.fdp
	$(ENSURE_TARGET_DIR)
	$(FDP) -Tfig $< -o $@.tmp
## There is a bug in fdp's fig output, see Debian bug 373005.
	perl -0777 -pi -e 's/^2 3 0.*\n.*\n//m' $@.tmp
	mv $@.tmp $@

%.pdf: %.fig
	$(ENSURE_TARGET_DIR)
	$(FIG2DEV) -Lpdf -p dummy $< $@.tmp
	mv $@.tmp $@

%.eps: %.fig
	$(ENSURE_TARGET_DIR)
	$(FIG2DEV) -Leps -p dummy $< $@.tmp
	mv $@.tmp $@


## pdftex/pdftex_t combined XFig pictures.
##
## To avoid problems, the PDF file should end with .pdf, not .pdftex
## as suggested in xfig documentation.  And to avoid ambiguity on
## *.fig -> *.pdf, let's use *.pdffig as input extension instead of
## *.fig.
##
## A single Make rule with two commands because it simplifies the use:
## depend on one file, not two.  Generate the PDF last, since it bears
## the dependency.
%.pdf %.eps %.pdftex_t: %.pdffig
	$(ENSURE_TARGET_DIR)
	-rm -f $*.{pdf,pdftex_t}
	$(FIG2DEV) -Lpdftex_t -p $*.pdf $< $*.pdftex_t
# Some versions of fig2dev produce code for epsfig instead of graphicx.
# Do not force the extension to PDF.
	perl -pi -e 's/\\epsfig\{file=/\\includegraphics{/;' \
		 -e 's/(\\includegraphics\{.*)\.pdf/$$1/;'   \
	  $*.pdftex_t
	$(FIG2DEV) -Lpdftex -p dummy $< $*.pdf
	$(FIG2DEV) -Lpstex -p dummy $< $*.eps


## pdftex/pdftex_t combined GNU Plot pictures.
##
## The GNU Plot file needs not set the output file name, nor the terminal:
## set are properly set by default.
##
## A single Make rule with two commands because it simplifies the use:
## depend on one file, not two.
%.pdftex_t %.eps %.pdf: %.plt
	$(ENSURE_TARGET_DIR)
# gnuplot creates an output, even if it failed.  Remove it.
	rm -f $*.{tex,eps,pdf,pdftex_t}
# Put the output first, before the plotting command, and before
# requests from the user.  Set the terminal too there for the same
# reasons.
	{					\
	  echo 'set output "$*.eps"' &&		\
	  echo 'set terminal epslatex color' &&	\
	  cat $<;				\
	} > $*.plt.tmp
	LC_ALL=C $(GNUPLOT) $*.plt.tmp
	mv $*.tex $*.pdftex_t
	$(EPSTOPDF) $*.eps -o $*.pdf
	rm $*.plt.tmp

%.pdf: %.gif
	$(CONVERT_TO_PDF)
%.pdf: %.id
	$(ENSURE_TARGET_DIR)
	$(EPSTOPDF) $< -o pdf:$@.tmp
	mv $@.tmp $@

#%.pdf: %.eps
#	$(ENSURE_TARGET_DIR)
#	$(EPSTOPDF) $< -o $@.tmp
#	mv $@.tmp $@

%.pdf: %.jpg
	$(CONVERT_TO_PDF)
%.pdf: %.png
	$(CONVERT_TO_PDF)
%.pdf: %.tif
	$(CONVERT_TO_PDF)
%.pdf: %.tiff
	$(CONVERT_TO_PDF)
%.pdf: %.pbm
	$(CONVERT_TO_PDF)
%.pdf: %.pgm
	$(CONVERT_TO_PDF)
%.pdf: %.ppm
	$(CONVERT_TO_PDF)

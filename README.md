# Tutorial for Jackal

---

**Note:**
These tutorials have been superceded. Find the latest Boxer tutorials at [docs.clearpathrobotics.com](https://docs.clearpathrobotics.com/docs/robots/jackal/tutorials_jackal).

---

## Setup
    sudo apt-get install texlive-full texlive-xetex texmaker python-pip latexmk
    pip3 install sphinx

## Download
When downloading this repository, we also get a CP theme for the web version to make it look pretty.  The easiest way
to get this it to clone recursively by using the --recursive flag.  Alternatively, you can initialize the submodule
after cloning by running

    git submodule init
    git submodule update

## Build Webpages
    sphinx-build -b html doc html

## Build PDF
    sphinx-build -b latex doc latex
    cd latex/
    make

If you get errors about missing fonts, go to latex/clearpath-latex/fonts and install all the fonts there

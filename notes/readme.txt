You'll need to have svn, gnuplot and pdflatex installed on your machine.

IPE is also recommended for editing figures.

To install these for Mac OS:
- Download and install MacTex: http://www.tug.org/mactex/2011/
- Download and install MacPorts: http://www.macports.org/
- Run the following commands:
-- sudo port install gnuplot
-- sudo port install subversion <- Good luck viewing this repo without svn!
-- sudo port install ipe
-- sudo port install librsvg

To build the paper, cd to the tex directory and type:
make

To automatically open the resultant pdf, type
make view

The paper directory structure is as follows:

-  tex   - Contains the paper text, bibliography, and output PDF.
           Type make within this directory to cause the entire paper
           to build, this will additionally build the figs and images
           directory.
-  figs  - Contains figures. Recommended layout is to create
           a separate subdirectory for each experiment, and 
           put both the data, gnuplot files (*.plt), and any
           processing utilities. Additionally put a copy of the
           makefile of the root of this directory in each subdir.
- images - Useful for diagrams and jpegs.



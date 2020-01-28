# -*- coding: utf-8 -*-

import os
import sys
import xml.etree.ElementTree as etree

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.extlinks',
    'sphinx.ext.doctest',
    'sphinx.ext.viewcode',
]

extlinks = {
    'roswiki': ('http://wiki.ros.org/%s', ''),
}

source_suffix = '.rst'
master_doc = 'index'

project = u'Jackal Tutorials'
copyright = u'2015, Clearpath Robotics'

# Get version number from package.xml.
tree = etree.parse('../package.xml')
version = tree.find("version").text
release = version

html_theme = 'theme'
html_theme_path = ["."]

htmlhelp_basename = 'jackal_tutorialsdoc'
templates_path = ['./templates']
html_static_path = ['./theme/static']

html_sidebars = {
   '**': ['sidebartoc.html', 'sourcelink.html', 'searchbox.html']
}

rst_prolog = """
.. |ros_distro| replace:: kinetic
.. |ubuntu_distro| replace:: xenial
"""
#.. ubuntu_distro: xenial

# If true, "Created using Sphinx" is shown in the HTML footer. Default is True.
html_show_sphinx = False

# The name of an image file (relative to this directory) to place at the top
# of the sidebar.
html_logo = 'clearpathlogo.png'

# The name of an image file (within the static path) to use as favicon of the
# docs.  This file should be a Windows icon file (.ico) being 16x16 or 32x32
# pixels large.
html_favicon = 'favicon.ico'

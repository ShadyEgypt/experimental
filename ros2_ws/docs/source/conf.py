import os
import sys

sys.path.insert(0, os.path.abspath('../../src'))

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'ex_assignment1'
copyright = '2025, Shady'
author = 'Shady'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx_autodoc_typehints',
    "breathe",
]

autodoc_mock_imports = [
    "cv2",
    "cv_bridge",
    "rclpy",
]

templates_path = ['_templates']
exclude_patterns = []

breathe_projects = {
    "Experimental": "../../xml"
}

breathe_default_project = "Experimental"

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

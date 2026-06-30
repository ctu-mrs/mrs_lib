# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html


import os


def check_get_env(name: str) -> str:
    val = os.environ.get(name)
    if val is None:
        raise ValueError(
            "Must set environment variable 'MRS_LIB_DOCS_DOXYGEN_XML_OUTPUT'"
        )
    return val


doxygen_xml_dir = check_get_env("MRS_LIB_DOCS_DOXYGEN_XML_OUTPUT")
doxygen_html_extra_dir = check_get_env("MRS_LIB_DOCS_DOXYGEN_HTML_EXTRA_DIR")

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "mrs_lib"
copyright = "2026, Multi-robot Systems (MRS) group at Czech Technical University in Prague"
author = (
    "Multi-robot Systems (MRS) group at Czech Technical University in Prague"
)

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ["sphinx.ext.autodoc", "breathe"]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_extra_path = [doxygen_html_extra_dir]


breathe_projects = {"mrs_lib": doxygen_xml_dir}
breathe_default_project = "mrs_lib"

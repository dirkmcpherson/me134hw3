from fontTools.ttLib import TTFont
from IPython import embed
import svglib

path = "./fonts/otf/Urbanist-Thin.otf"
font = TTFont(path)

embed()

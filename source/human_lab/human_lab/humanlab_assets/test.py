import os

# Conveniences to other module directories via relative paths
ISAACLAB_ASSETS_EXT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../"))
"""Path to the extension source directory."""

ISAACLAB_ASSETS_DATA_DIR = os.path.join(ISAACLAB_ASSETS_EXT_DIR, "humanlab_assets/assets")

if __name__ == '__main__':

   path1 =  ISAACLAB_ASSETS_EXT_DIR
   path2 = ISAACLAB_ASSETS_DATA_DIR
   ptth3 = path1
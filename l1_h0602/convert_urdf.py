"""Convert h0612.urdf to USD using Isaac Sim 6's urdf_usd_converter."""
import os

from urdf_usd_converter import Converter

HERE = os.path.dirname(os.path.abspath(__file__))
INPUT = os.path.join(HERE, "urdf", "h0612.urdf")
OUTPUT_DIR = os.path.join(HERE, "usd")

os.makedirs(OUTPUT_DIR, exist_ok=True)

converter = Converter(layer_structure=True, scene=True)
asset_path = converter.convert(INPUT, OUTPUT_DIR)

print(f"USD_ASSET_PATH={asset_path.path}")

"""Reuse the verified STL preview renderer with measured-panel revision labels."""
from pathlib import Path
root=Path(__file__).resolve().parent
source=(root/'render_mount.py').read_text()
changes={
    "OUT=ROOT/'output'":"OUT=ROOT/'output_v2'",
    "'mount_dimensions_and_checks.json'":"'dimensions_and_checks_v2.json'",
    "'LidTop_Adafruit6383_mount_PROTOTYPE.stl'":"'LidTop_measured_panel_mount_v2.stl'",
    "'added_features.stl'":"'added_features_v2.stl'",
    "'rear_assembled.stl'":"'rear_assembled_v2.stl'",
    "'panel_reference_DO_NOT_PRINT.stl'":"'panel_reference_v2.stl'",
    "'Adafruit6383_mount_fit_coupon.stl'":"'Mount_fit_coupon_v2.stl'",
    "'E-ink mount for your lid'":"'Revised mount - measured screen outline'",
    "'Adafruit 6383 | Prototype for fit testing | Dimensions in mm'":"'Panel: 79.05 x 36.71 x 0.95 mm | Revision 2 | Prototype for fit testing'",
    "'Added seat, guides and four screw bosses'":"'Wider guides, border supports and screw bosses'",
    "'Pocket: 29.8 x 59.8'":"'Pocket: 80.05 x 37.71 mm'",
    "'Viewing opening: 25.3 x 50.15'":"'Original slot retained: 55 x 31.9 mm'",
    "'mount_preview.png'":"'mount_preview_v2.png'",
    "'fit_coupon_preview.png'":"'fit_coupon_preview_v2.png'",
}
for old,new in changes.items():
    assert old in source,old
    source=source.replace(old,new)
(root/'render_mount_v2.py').write_text(source)
exec(compile(source,str(root/'render_mount_v2.py'),'exec'),{'__file__':str(root/'render_mount_v2.py'),'__name__':'__main__'})

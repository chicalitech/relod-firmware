# Adafruit 6383 e-ink mount - prototype

Print the small fit coupon and rear retainer first, in millimeters at 100% scale. The mesh checks pass, but the fit, screw engagement, and pad compression have not been physically tested.

## Files

- `LidTop_Adafruit6383_mount_PROTOTYPE.stl`: complete modified lid, with an integrated seating bezel, side guides, two lower stops and four screw bosses.
- `Adafruit6383_rear_retainer_PROTOTYPE.stl`: separate U-shaped rear frame; print flat as exported.
- `Adafruit6383_mount_fit_coupon.stl`: small section of the lid for checking the same mounting features before a full print.
- `mount_preview.png`: inside and exploded views. The gold panel is a reference envelope; foam and screws are not illustrated.
- `mount_dimensions_and_checks.json`: dimensions, source fingerprint and mesh validation.

## Fit and orientation

Designed for Adafruit product 6383, bare panel ZJY122250-0213BAAMFGN. The body is 29.2 x 59.2 x 1.0 mm; the mechanical drawing gives +/-0.2 mm outline and +/-0.1 mm thickness tolerances. The pocket is 29.8 x 59.8 mm (0.3 mm nominal clearance per side). Maximum panel outline still leaves 0.2 mm per side before print tolerances.

The original cutout was 31.9 x 55 mm, wider than the panel. Added material narrows the through-opening to 25.3 x 50.15 mm and supports the glass border. Its center matches the original opening. The panel body is offset 2.625 mm toward the ribbon end so the active area remains centered. Place the ribbon toward negative Y: the bottom in the inside-view preview. A lower central relief avoids seating against the driver/bond region; the retainer is open at that end.

The source STL was interpreted in millimeters. Its overall bounds are about 127.71 x 127.72 x 22.21 mm. The original file was not overwritten. This is an STL mesh edit, not a native SolidWorks feature tree. Existing source material was retained by the construction; output simplification is limited to a 0.0001 mm tolerance.

## Assembly

1. Remove printing burrs. Try the pocket with a 29.2 x 59.2 mm card or plastic blank before placing the glass. The display must drop in freely; do not flex it into the guides.
2. Apply thin soft cushioning strips along the two long seating borders. Keep the strips outside the viewing opening and away from the ribbon/driver region. Aim for roughly 1 mm strip width and 35-40 mm length, beginning about 10 mm above the ribbon end. The design allows 0.30 mm installed front cushioning thickness.
3. Place the display face down toward the viewing opening, ribbon toward the open lower end. Its front face sits at Z=3.005 mm with nominal front cushioning.
4. Apply matching soft strips to the underside of the retainer directly opposite the front strips. The nominal rear gap is 0.50 mm. Select cushioning that lightly takes up the actual gap; do not force a thick stack or use the screws to compress the display. The cavity between seat and retainer is 1.80 mm total.
5. Fit the retainer over the four bosses. Its underside stops at Z=4.505 mm on the bosses and guide walls. Use four M2.5 x 5 mm pan-head thread-forming screws intended for plastic, each with a 0.5 mm-thick washer (about 6 mm outside diameter), subject to the screw maker's pilot recommendation. The prototype has 2.0 mm blind pilots and 2.8 mm retainer clearance holes. With the washer, a 5 mm screw projects about 2.1 mm beyond the 2.4 mm retainer into a 2.6 mm-deep pilot. Verify actual screw length; the washers provide tip clearance. Do not substitute a longer screw without checking depth. Test screws in the coupon without the display first.
6. Verify that the retainer reaches its hard stops with light screw tension and that the panel is held without bending. Adjust cushioning for the measured panel thickness and printed dimensions. Route the FPC through the open end without trapping or creasing it.

The complete lid and fit coupon are exported with the exterior face on Z=0. Inspect the slicer preview for your material and machine. Enclosure assembly clearance beyond the supplied lid, sealing, impact resistance, and temperature performance were not tested.

## Reference

[Adafruit product 6383](https://www.adafruit.com/product/6383). The product's datasheet link returned an error; the same model's [manufacturer drawing hosted by Adafruit](https://cdn-shop.adafruit.com/product-files/4197/C13256-007_datasheet_ZJY122250-0213BAAMFGN.pdf), pages 4-5, supplies the mechanical dimensions.

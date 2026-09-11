# Measured-panel mount, revision 2

This revision uses your measured rigid panel outline of **79.05 x 36.71 x 0.95 mm**, excluding the ribbon. It supersedes the earlier catalog-based mount. Use the v2 lid and v2 retainer together; the first retainer has different screw locations.

## What changed

- Pocket enlarged to **80.05 x 37.71 mm**, with **0.5 mm clearance per side**.
- Original **55 x 31.9 mm viewing slot preserved**, including its position. No material is added inside that opening.
- Full panel outline centered on the original viewing slot, with its long dimension along the slot's long dimension. Active-area offsets have not been measured; check image alignment during the trial fit.
- Two narrow ledges support the panel's long borders through soft cushioning. The enlarged guides locate the entire panel, not the viewing area.
- Four screw bosses repositioned above and below the neighboring sensor feature.
- Retainer height set for the measured **0.95 mm glass** plus cushioning.
- Ribbon exits the open short end toward the lower lid edge in the preview. The center of the lower edge stays open; two corner stops support the panel's lower edge.

## Files to print

1. **Mount_fit_coupon_v2.stl** - print this approximately 54.5 x 85.25 mm fit piece first. It reproduces the guides, seating ledges and screw bosses over a flat section of lid, without the unrelated enclosure details.
2. **Rear_retainer_measured_panel_v2.stl** - print flat as exported. Test it with the coupon.
3. **LidTop_measured_panel_mount_v2.stl** - complete replacement lid, to print after checking the coupon.

Import in **millimeters, at 100% scale**. Remove burrs before fitting the display. The output is a mesh STL, not a native SolidWorks feature tree. Your original supplied STL remains unchanged.

## Assembly

1. Check that the whole rigid screen outline drops freely between the guides. Do not bend or force the glass. The nominal guide opening is 37.71 mm across and 80.05 mm along the panel.
2. Put thin soft strips on the two narrow raised seating ledges, outside the original viewing slot. Each ledge is 1.4 mm wide and about 66 mm long, beginning 10 mm above the ribbon end. Keep pads within those ledges and away from the driver/bond region. The design allows **0.30 mm installed front cushioning**.
3. Place the screen face down toward the outside of the lid, with its ribbon toward the open short end at the bottom of the inside-view preview. Check the visible image's alignment with the original slot before fastening.
4. Put matching soft strips under the two sides of the rear retainer, directly opposite the seating strips. The nominal rear gap is **0.50 mm**. Select pads to lightly take up the actual gap; do not use the screws to force a thick pad stack against the glass. The seat-to-retainer cavity is 1.75 mm total.
5. Place the v2 retainer on the v2 guide walls and screw bosses. These are fixed stops that set the retainer height.
6. Insert **four M2.5 x 5 mm pan-head thread-forming screws for plastic**, each with a **0.5 mm-thick washer** (about 6 mm outer diameter), from inside the lid. Screws pass through the retainer's 2.8 mm clearance holes into the bosses' 2.0 mm blind pilots. Check the screw manufacturer's pilot recommendation and test screw engagement on the coupon without the glass. With a 2.4 mm retainer and 0.5 mm washer, a 5 mm screw engages about 2.1 mm into a 2.55 mm-deep pilot. Verify actual hardware dimensions; avoid bottoming out.
7. Tighten gently to the hard stops. The panel should be retained without bowing. Adjust cushioning if needed, then route the ribbon through the open end without pinching it.

## Checks and limits

The three exported STLs are watertight, consistently oriented, and each contains one connected solid. A panel envelope expanded by 0.1 mm at each edge and by 0.1 mm in thickness clears the modeled lid and retainer. Straight insertion from the enclosure interior is clear. The original slot remains unobstructed, and no original lid material is removed by the construction. The numerical mesh cleanup tolerance is 0.0001 mm.

These are geometric checks, not a physical fit test. Print tolerances, cushioning and screw fit still need checking. The active-region position within your measured panel has not been supplied; this revision centers the panel's outer outline on your existing slot. The complete enclosure assembly, ribbon bend path beyond the lid, sealing and impact resistance have not been tested.

See **dimensions_and_checks_v2.json** for the coordinates and validation values. In **mount_preview_v2.png**, teal is the added support geometry, gold is the panel reference envelope, and blue is the rear retainer. Foam, screws and the flexible ribbon are omitted from the exploded view.

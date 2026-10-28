# FM-1 sidebar-fit audit (status: phase 1 done)
Method: one offscreen run (tools: scratchpad audit.py), MainWindow in Advanced mode, `_refresh_steps` stubbed,
panels.setFixedWidth(420), 1600x950. Viewport inner width 406 (Part/Position, vertical scrollbar) / 420; root layout
margins 9+9 so usable ~388-402 px. Listed = deepest visible widget whose minimumSizeHint/sizeHint width > viewport.
Offscreen font metrics are approximate (U24-2: glyphs render as boxes); U24 PNGs confirm the same overflows on screen.
Root min width per panel: Part 538, Position 582, Toolpath 1740, Simulate 489, Export 302 (434 hint) -> all but Export overflow.

| Step | Widget (file) | min / hint px | Fix |
|---|---|---|---|
| Part | Test-part QComboBox (part.py) | 474 | combo: setSizeAdjustPolicy(AdjustToMinimumContentsLengthWithIcon), setMinimumContentsLength(12), Expanding |
| Part | Bodies section row: "Isolate build body" + "Show all" capsules (part.py capsule_row) | 246+126+8 + margins | shorten to "Isolate build" / "Show all", capsules Expanding with min width 0; or stack |
| Part | Body row: QCheckBox "Body 1: 40 x 30 x 5 mm" + role combo Build/Fixture/Ignore | 288+114 | put the dims on a second line (wrap label), role combo AdjustToContents |
| Position | Placement grid 2x2: Snap base face down / Drop onto plate / Centre on faceplate / Reset (position.py:95) | 476 | 1 column (stack vertically) or a "Place..." QToolButton with a dropdown menu of the 4 actions |
| Position | QCheckBox "SpaceNavigator for Notebooks moves the part" | 540 | shorter text "SpaceMouse moves the part" + tooltip for the device name |
| Position | Injector QComboBox "Central Injector Kit (CIK) (180 mm)..." | 450 | combo AdjustToMinimumContentsLengthWithIcon + popup view minimum width = content |
| Position | Purge form (position.py:501): Direction combo "Down-angled (-Y, 30 deg down)" 342 + label column 108; spins 132-168; buttons 218-242 | 456 | QFormLayout.setRowWrapPolicy(WrapLongRows), direction combo min-contents-length, spins setMinimumWidth(0)/Expanding, the 3 buttons in one "Purge actions" dropdown or stacked full width |
| Toolpath | Mode SegmentedCapsule with 3 long labels "Layers (build up...)", "Revolve (...)", "Surface (...)" (toolpath.py:173,265) | 500/524/596 each -> 1740 | DROPDOWN: show the QComboBox (it is already the data API), drop the capsule; or capsule with short labels Layers/Revolve/Surface + long text as tooltip |
| Toolpath | StatTiles (3 tiles in one QHBoxLayout, toolpath.py:204) | ~350 (masked by mode row; U24-2 saw it cut) | QGridLayout 2 per row, tiles Expanding |
| Simulate | Stat tile row, 4 tiles (simulate.py:90) | 447 (92+125+136+70+gaps) | 2x2 QGridLayout |
| Simulate | PlaybackControls row (timeline.py:283): play/speed/legend checkboxes + Follow tool | 402/415 | two rows (transport + speed on row 1, legend + Follow on row 2), speed combo short items |
| All | Global style: QPushButton padding 0 14px, QComboBox padding-right 22, spin padding-right 20 (style.py:351-379) | - | smaller horizontal padding (10px), QComboBox min-contents behaviour in a helper |
| Export | fits (302 min) | - | none |

App level (owned by U21, check after merge): PANEL_WIDTH 380 -> 420, sidebar scroll areas ScrollBarAlwaysOff horizontally.

## Fix split (disjoint)
- F1 part.py, position.py, constraints_panel.py: Part combo/body rows, Position placement buttons (stack or dropdown), SpaceNavigator text, injector combo, purge form wrap + actions.
- F2 toolpath.py, simulate.py, export.py, timeline.py: Toolpath mode capsule -> QComboBox dropdown, StatTiles 2-col grid, Simulate tiles 2x2, PlaybackControls 2 rows.
- F3 widgets.py, style.py: widgets.fit_combo(combo, n=12) helper, widgets.Segmented(combo, max_width) that shows the combo when the row would not fit, widgets.TileGrid(cols=2); trim padding in style.py.

## Stopped 08:50 (user decision): F1-F3 NOT merged into wp/FM
State: wp/FM = 3f62b53 (audit + U20/U21/U23). F branches kept for next session: wp/F3 7e8a80f (done, 68 passed incl. U20 test fix),
wp/F2fit b08a4df (done: toolpath/simulate/export fit + export split_krl/preflight, 4 + krl_regression passed),
wp/F1 (no commit yet at 08:50). Old wp/F2 and wp/F2b are unrelated; ignore them.
Merge order: wp/F3 -> wp/F1 -> wp/F2fit (F3 first: fit_combo/TileGrid in widgets.py that F1/F2fit may use).
Steps left:
1. Merge the three in that order (F3 and F2fit merged cleanly onto 3f62b53 in a trial; reverted).
2. Test offscreen: test_gui_layout, test_imports, test_timeline_legend, test_ui_mode_steps, test_u20_glass_light,
   test_krl_regression, test_f3_widgets, test_sidebar_fit_f2, plus F1's tests.
3. Merge main (has U20/U21/U23, KM, OP1/OP2 already): expect mostly no-ops; likely conflict in steps/export.py
   (F2fit split/preflight vs main's KM merge).
4. Re-audit once, check every panel's root minimumSizeHint <= ~400 px.
Audit-script note: scratchpad audit.py (copy to tools/ if kept) uses panels.widget(i) as the QScrollArea; since U21 each
page is a wrapper, so use panels.widget(i).scroll (and .scroll.viewport() for the inner width). Offscreen font
metrics are approximate; it reports the deepest visible widget whose min/size hint exceeds the viewport.

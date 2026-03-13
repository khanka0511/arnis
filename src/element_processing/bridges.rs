use crate::block_definitions::*;
use crate::bresenham::bresenham_line;
use crate::osm_parser::{ProcessedNode, ProcessedWay};
use crate::world_editor::WorldEditor;

/// Visual configuration for a bridge, determined by its OSM bridge type tag.
pub struct BridgeConfig {
    pub deck_block: Block,
    pub railing_block: Block,
    pub support_block: Block,
    pub has_railings: bool,
    pub support_interval: i32,
    /// Half-width of the bridge deck (e.g. 2 means 5 blocks wide: -2..=2)
    pub half_width: i32,
}

impl BridgeConfig {
    /// Create a bridge configuration from the OSM `bridge=*` tag value.
    pub fn from_bridge_type(bridge_type: &str) -> Self {
        match bridge_type {
            "aqueduct" => Self {
                deck_block: STONE_BRICKS,
                railing_block: STONE_BRICK_WALL,
                support_block: STONE_BRICKS,
                has_railings: true,
                support_interval: 6,
                half_width: 2,
            },
            "boardwalk" => Self {
                deck_block: OAK_PLANKS,
                railing_block: OAK_FENCE,
                support_block: OAK_LOG,
                has_railings: true,
                support_interval: 4,
                half_width: 1,
            },
            "viaduct" => Self {
                deck_block: STONE_BRICKS,
                railing_block: STONE_BRICK_WALL,
                support_block: STONE_BRICKS,
                has_railings: true,
                support_interval: 6,
                half_width: 3,
            },
            "covered" => Self {
                deck_block: OAK_PLANKS,
                railing_block: OAK_PLANKS,
                support_block: OAK_LOG,
                has_railings: true,
                support_interval: 5,
                half_width: 2,
            },
            _ => Self {
                // Default for "yes" and any other unrecognised value
                deck_block: LIGHT_GRAY_CONCRETE,
                railing_block: COBBLESTONE_WALL,
                support_block: STONE_BRICKS,
                has_railings: true,
                support_interval: 8,
                half_width: 2,
            },
        }
    }
}

/// Calculate a flat bridge deck Y level from the way's endpoint ground elevations.
/// Uses the maximum of start/end ground levels so the deck doesn't dip into valleys.
pub fn calculate_bridge_deck_y(editor: &WorldEditor, nodes: &[ProcessedNode]) -> Option<i32> {
    if nodes.len() < 2 {
        return None;
    }
    let start = &nodes[0];
    let end = &nodes[nodes.len() - 1];
    let start_y = editor.get_ground_level(start.x, start.z);
    let end_y = editor.get_ground_level(end.x, end.z);
    Some(start_y.max(end_y))
}

/// Place railings along one side of a bridge segment.
///
/// `offset` is the perpendicular distance from the centre line (positive or negative).
/// The direction vector `(dir_x, dir_z)` is the normalised segment direction so we
/// can compute the perpendicular.
pub fn place_railings_for_segment(
    editor: &mut WorldEditor,
    points: &[(i32, i32, i32)],
    deck_y: i32,
    config: &BridgeConfig,
    dir_x: f64,
    dir_z: f64,
) {
    // Perpendicular direction (rotate 90 degrees)
    let perp_x = -dir_z;
    let perp_z = dir_x;

    for (x, _, z) in points {
        for &side in &[-1.0_f64, 1.0_f64] {
            let offset = (config.half_width as f64 + 1.0) * side;
            let rx = (*x as f64 + perp_x * offset).round() as i32;
            let rz = (*z as f64 + perp_z * offset).round() as i32;
            editor.set_block_absolute(config.railing_block, rx, deck_y + 1, rz, None, None);
        }
    }
}

/// Place a support pillar from ground level up to the bridge deck.
pub fn place_support_pillar(
    editor: &mut WorldEditor,
    x: i32,
    deck_y: i32,
    z: i32,
    config: &BridgeConfig,
) {
    let ground_y = editor.get_ground_level(x, z);
    if deck_y <= ground_y {
        return;
    }

    // Pillar column
    for y in (ground_y + 1)..deck_y {
        editor.set_block_absolute(config.support_block, x, y, z, None, None);
    }

    // Pillar base (3x3)
    for dx in -1..=1 {
        for dz in -1..=1 {
            editor.set_block_absolute(config.support_block, x + dx, ground_y, z + dz, None, None);
        }
    }
}

/// Generates a standalone bridge for ways that have a `bridge=*` tag but no
/// primary infrastructure tag (highway, railway, etc.).
///
/// This handles the uncommon but valid case of bridge-only ways in OSM data.
pub fn generate_bridges(editor: &mut WorldEditor, element: &ProcessedWay) {
    let bridge_type = match element.tags.get("bridge") {
        Some(bt) if bt != "no" => bt.clone(),
        _ => return,
    };

    if element.nodes.len() < 2 {
        return;
    }

    let config = BridgeConfig::from_bridge_type(&bridge_type);

    let deck_ground_y = match calculate_bridge_deck_y(editor, &element.nodes) {
        Some(y) => y,
        None => return,
    };

    // Elevate the deck above ground: use layer tag if present, otherwise default +3
    let layer_offset = element
        .tags
        .get("layer")
        .and_then(|l| l.parse::<i32>().ok())
        .unwrap_or(1)
        .max(1);
    let bridge_height = layer_offset * 3;
    let deck_y = deck_ground_y + bridge_height;

    // Track total length for support pillar placement
    let mut block_counter: i32 = 0;

    for i in 1..element.nodes.len() {
        let prev = &element.nodes[i - 1];
        let cur = &element.nodes[i];

        let points = bresenham_line(prev.x, 0, prev.z, cur.x, 0, cur.z);

        // Compute direction for perpendicular offset (railings)
        let seg_dx = (cur.x - prev.x) as f64;
        let seg_dz = (cur.z - prev.z) as f64;
        let seg_len = (seg_dx * seg_dx + seg_dz * seg_dz).sqrt();
        let (dir_x, dir_z) = if seg_len > 0.0 {
            (seg_dx / seg_len, seg_dz / seg_len)
        } else {
            (1.0, 0.0)
        };

        for (x, _, z) in &points {
            // Place deck
            for dx in -config.half_width..=config.half_width {
                editor.set_block_absolute(config.deck_block, *x + dx, deck_y, *z, None, None);
                // Foundation layer underneath
                editor.set_block_absolute(
                    config.support_block,
                    *x + dx,
                    deck_y - 1,
                    *z,
                    None,
                    None,
                );
            }

            // Support pillars at interval
            if block_counter % config.support_interval == 0 {
                place_support_pillar(editor, *x, deck_y - 1, *z, &config);
            }

            block_counter += 1;
        }

        // Railings
        if config.has_railings {
            place_railings_for_segment(editor, &points, deck_y, &config, dir_x, dir_z);
        }
    }
}

/// Adds railings to an existing highway bridge segment.
///
/// Called from `highways.rs` for ways that have `bridge=yes` and are elevated.
/// `points` are the Bresenham points for one segment of the bridge,
/// `deck_y` is the absolute Y of the road surface, and `block_range` is the
/// highway half-width.
pub fn add_highway_bridge_railings(
    editor: &mut WorldEditor,
    points: &[(i32, i32, i32)],
    deck_y: i32,
    block_range: i32,
    dir_x: f64,
    dir_z: f64,
) {
    let perp_x = -dir_z;
    let perp_z = dir_x;

    for (x, _, z) in points {
        for &side in &[-1.0_f64, 1.0_f64] {
            let offset = (block_range as f64 + 1.0) * side;
            let rx = (*x as f64 + perp_x * offset).round() as i32;
            let rz = (*z as f64 + perp_z * offset).round() as i32;
            editor.set_block_absolute(
                COBBLESTONE_WALL,
                rx,
                deck_y + 1,
                rz,
                None,
                Some(&[COBBLESTONE_WALL]),
            );
        }
    }
}

/// Adds bridge elevation support to a railway way that has `bridge=yes`.
///
/// Returns `Some((deck_y, config))` if this railway should be elevated as a bridge,
/// or `None` if it is not a bridge.
pub fn get_railway_bridge_info(
    editor: &WorldEditor,
    element: &ProcessedWay,
) -> Option<(i32, BridgeConfig)> {
    let bridge_type = element.tags.get("bridge")?;
    if bridge_type == "no" {
        return None;
    }

    if element.nodes.len() < 2 {
        return None;
    }

    let config = BridgeConfig::from_bridge_type(bridge_type);
    let deck_ground_y = calculate_bridge_deck_y(editor, &element.nodes)?;

    let layer_offset = element
        .tags
        .get("layer")
        .and_then(|l| l.parse::<i32>().ok())
        .unwrap_or(1)
        .max(1);
    let bridge_height = layer_offset * 3;
    let deck_y = deck_ground_y + bridge_height;

    Some((deck_y, config))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::coordinate_system::cartesian::XZBBox;
    use crate::coordinate_system::geographic::LLBBox;
    use std::collections::HashMap;

    /// Helper to create a WorldEditor for testing
    fn test_editor(xzbbox: &XZBBox) -> WorldEditor {
        let llbbox = LLBBox::new(54.0, 9.0, 55.0, 10.0).unwrap();
        WorldEditor::new(
            std::path::PathBuf::from("/tmp/arnis_test_bridge"),
            xzbbox,
            llbbox,
        )
    }

    /// Helper to create a ProcessedWay with bridge tags
    fn make_bridge_way(
        bridge_type: &str,
        nodes: Vec<(i32, i32)>,
        extra_tags: Vec<(&str, &str)>,
    ) -> ProcessedWay {
        let mut tags = HashMap::new();
        tags.insert("bridge".to_string(), bridge_type.to_string());
        for (k, v) in extra_tags {
            tags.insert(k.to_string(), v.to_string());
        }
        ProcessedWay {
            id: 1,
            nodes: nodes
                .into_iter()
                .enumerate()
                .map(|(i, (x, z))| ProcessedNode {
                    id: i as u64,
                    tags: HashMap::new(),
                    x,
                    z,
                })
                .collect(),
            tags,
        }
    }

    // --- BridgeConfig tests ---

    #[test]
    fn test_config_default_bridge() {
        let config = BridgeConfig::from_bridge_type("yes");
        assert_eq!(config.deck_block, LIGHT_GRAY_CONCRETE);
        assert_eq!(config.railing_block, COBBLESTONE_WALL);
        assert_eq!(config.support_block, STONE_BRICKS);
        assert!(config.has_railings);
        assert_eq!(config.half_width, 2);
    }

    #[test]
    fn test_config_aqueduct() {
        let config = BridgeConfig::from_bridge_type("aqueduct");
        assert_eq!(config.deck_block, STONE_BRICKS);
        assert_eq!(config.railing_block, STONE_BRICK_WALL);
        assert!(config.has_railings);
    }

    #[test]
    fn test_config_boardwalk() {
        let config = BridgeConfig::from_bridge_type("boardwalk");
        assert_eq!(config.deck_block, OAK_PLANKS);
        assert_eq!(config.railing_block, OAK_FENCE);
        assert_eq!(config.support_block, OAK_LOG);
        assert_eq!(config.half_width, 1);
    }

    #[test]
    fn test_config_viaduct() {
        let config = BridgeConfig::from_bridge_type("viaduct");
        assert_eq!(config.deck_block, STONE_BRICKS);
        assert_eq!(config.half_width, 3);
        assert_eq!(config.support_interval, 6);
    }

    #[test]
    fn test_config_covered() {
        let config = BridgeConfig::from_bridge_type("covered");
        assert_eq!(config.deck_block, OAK_PLANKS);
        assert_eq!(config.support_block, OAK_LOG);
    }

    #[test]
    fn test_config_unknown_type_uses_default() {
        let config = BridgeConfig::from_bridge_type("some_future_type");
        assert_eq!(config.deck_block, LIGHT_GRAY_CONCRETE);
        assert!(config.has_railings);
    }

    // --- calculate_bridge_deck_y tests ---

    #[test]
    fn test_deck_y_needs_two_nodes() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let editor = test_editor(&xzbbox);
        let nodes = vec![ProcessedNode {
            id: 0,
            tags: HashMap::new(),
            x: 10,
            z: 10,
        }];
        assert!(calculate_bridge_deck_y(&editor, &nodes).is_none());
    }

    #[test]
    fn test_deck_y_returns_max_of_endpoints() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let editor = test_editor(&xzbbox);
        // Without ground data, get_ground_level returns 0 for all points
        let nodes = vec![
            ProcessedNode {
                id: 0,
                tags: HashMap::new(),
                x: 10,
                z: 10,
            },
            ProcessedNode {
                id: 1,
                tags: HashMap::new(),
                x: 50,
                z: 50,
            },
        ];
        let y = calculate_bridge_deck_y(&editor, &nodes);
        assert_eq!(y, Some(0)); // Both endpoints at ground level 0 without terrain
    }

    #[test]
    fn test_deck_y_empty_nodes() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let editor = test_editor(&xzbbox);
        let nodes: Vec<ProcessedNode> = vec![];
        assert!(calculate_bridge_deck_y(&editor, &nodes).is_none());
    }

    // --- generate_bridges tests ---

    #[test]
    fn test_generate_bridges_skips_bridge_no() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let mut editor = test_editor(&xzbbox);
        let way = make_bridge_way("no", vec![(10, 10), (50, 10)], vec![]);
        // Should return early without placing any blocks
        generate_bridges(&mut editor, &way);
        // Verify no blocks placed at the midpoint deck position
        assert!(!editor.block_at_absolute(30, 3, 10));
    }

    #[test]
    fn test_generate_bridges_single_node_skips() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let mut editor = test_editor(&xzbbox);
        let way = make_bridge_way("yes", vec![(10, 10)], vec![]);
        generate_bridges(&mut editor, &way);
        // No crash, no blocks
        assert!(!editor.block_at_absolute(10, 3, 10));
    }

    #[test]
    fn test_generate_bridges_places_deck() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let mut editor = test_editor(&xzbbox);
        let way = make_bridge_way("yes", vec![(10, 50), (50, 50)], vec![]);
        generate_bridges(&mut editor, &way);
        // Default layer=1 -> bridge_height=3, deck_y=0+3=3
        // Check that deck blocks are placed at the midpoint
        assert!(editor.block_at_absolute(30, 3, 50));
    }

    #[test]
    fn test_generate_bridges_respects_layer_tag() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let mut editor = test_editor(&xzbbox);
        let way = make_bridge_way("yes", vec![(10, 50), (50, 50)], vec![("layer", "2")]);
        generate_bridges(&mut editor, &way);
        // layer=2 -> bridge_height=6, deck_y=0+6=6
        assert!(editor.block_at_absolute(30, 6, 50));
        // Should not have blocks at default layer=1 height
        assert!(!editor.block_at_absolute(30, 3, 50));
    }

    #[test]
    fn test_generate_bridges_places_foundation() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let mut editor = test_editor(&xzbbox);
        let way = make_bridge_way("yes", vec![(10, 50), (50, 50)], vec![]);
        generate_bridges(&mut editor, &way);
        // Foundation should be one block below deck (y=2)
        assert!(editor.block_at_absolute(30, 2, 50));
    }

    #[test]
    fn test_generate_bridges_boardwalk_narrower() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let mut editor = test_editor(&xzbbox);
        let way = make_bridge_way("boardwalk", vec![(10, 50), (50, 50)], vec![]);
        generate_bridges(&mut editor, &way);
        // Boardwalk has half_width=1. Deck extends via dx from each centre point.
        // The first centre point is x=10. With half_width=1, deck goes x=9,10,11.
        // x=8 should be empty (only reachable with half_width>=2).
        // Deck at y=3 (layer=1 default, bridge_height=3)
        assert!(editor.block_at_absolute(30, 3, 50)); // Centre point
        assert!(editor.block_at_absolute(9, 3, 50)); // 1 block before first centre (half_width=1)
        assert!(!editor.block_at_absolute(8, 3, 50)); // 2 blocks before first centre (would need half_width>=2)
    }

    // --- get_railway_bridge_info tests ---

    #[test]
    fn test_railway_bridge_info_none_without_tag() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let editor = test_editor(&xzbbox);
        let mut tags = HashMap::new();
        tags.insert("railway".to_string(), "rail".to_string());
        let way = ProcessedWay {
            id: 1,
            nodes: vec![
                ProcessedNode {
                    id: 0,
                    tags: HashMap::new(),
                    x: 10,
                    z: 10,
                },
                ProcessedNode {
                    id: 1,
                    tags: HashMap::new(),
                    x: 50,
                    z: 50,
                },
            ],
            tags,
        };
        assert!(get_railway_bridge_info(&editor, &way).is_none());
    }

    #[test]
    fn test_railway_bridge_info_returns_config() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let editor = test_editor(&xzbbox);
        let mut tags = HashMap::new();
        tags.insert("railway".to_string(), "rail".to_string());
        tags.insert("bridge".to_string(), "yes".to_string());
        let way = ProcessedWay {
            id: 1,
            nodes: vec![
                ProcessedNode {
                    id: 0,
                    tags: HashMap::new(),
                    x: 10,
                    z: 10,
                },
                ProcessedNode {
                    id: 1,
                    tags: HashMap::new(),
                    x: 50,
                    z: 50,
                },
            ],
            tags,
        };
        let info = get_railway_bridge_info(&editor, &way);
        assert!(info.is_some());
        let (deck_y, config) = info.unwrap();
        assert_eq!(deck_y, 3); // ground_y=0 + layer=1*3
        assert_eq!(config.deck_block, LIGHT_GRAY_CONCRETE);
    }

    #[test]
    fn test_railway_bridge_info_skips_bridge_no() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let editor = test_editor(&xzbbox);
        let mut tags = HashMap::new();
        tags.insert("railway".to_string(), "rail".to_string());
        tags.insert("bridge".to_string(), "no".to_string());
        let way = ProcessedWay {
            id: 1,
            nodes: vec![
                ProcessedNode {
                    id: 0,
                    tags: HashMap::new(),
                    x: 10,
                    z: 10,
                },
                ProcessedNode {
                    id: 1,
                    tags: HashMap::new(),
                    x: 50,
                    z: 50,
                },
            ],
            tags,
        };
        assert!(get_railway_bridge_info(&editor, &way).is_none());
    }

    // --- add_highway_bridge_railings tests ---

    #[test]
    fn test_highway_bridge_railings_places_blocks() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let mut editor = test_editor(&xzbbox);
        // Horizontal segment along z=50
        let points: Vec<(i32, i32, i32)> = (20..=40).map(|x| (x, 0, 50)).collect();
        let deck_y = 5;
        let block_range = 2;
        // Direction is along x-axis
        add_highway_bridge_railings(&mut editor, &points, deck_y, block_range, 1.0, 0.0);
        // Railings should be at z=50 +/- (block_range+1) = z=47 and z=53
        // at y = deck_y + 1 = 6
        assert!(editor.block_at_absolute(30, 6, 47));
        assert!(editor.block_at_absolute(30, 6, 53));
        // No railing in the centre
        assert!(!editor.block_at_absolute(30, 6, 50));
    }
}

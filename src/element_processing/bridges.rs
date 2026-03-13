use std::collections::HashMap;

use crate::block_definitions::*;
use crate::bresenham::bresenham_line;
use crate::osm_parser::{ProcessedNode, ProcessedWay};
use crate::world_editor::WorldEditor;

/// Bridge structural type, determined by the `bridge:structure` OSM tag.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BridgeStructure {
    /// Simple flat beam bridge (default)
    Beam,
    /// Arch bridge with curved underside
    Arch,
    /// Truss bridge with side bracing
    Truss,
    /// Suspension bridge with towers and draped cables
    Suspension,
    /// Cable-stayed bridge with diagonal cables from pylons
    CableStayed,
}

/// Visual configuration for a bridge, determined by OSM tags.
pub struct BridgeConfig {
    pub deck_block: Block,
    pub railing_block: Block,
    pub support_block: Block,
    pub has_railings: bool,
    pub support_interval: i32,
    /// Half-width of the bridge deck (e.g. 2 means 5 blocks wide: -2..=2)
    pub half_width: i32,
    pub structure_type: BridgeStructure,
}

impl BridgeConfig {
    /// Create a bridge configuration from the OSM `bridge=*` tag value only.
    #[cfg(test)]
    pub fn from_bridge_type(bridge_type: &str) -> Self {
        let mut tags = HashMap::new();
        tags.insert("bridge".to_string(), bridge_type.to_string());
        Self::from_tags(&tags)
    }

    /// Create a bridge configuration from the full set of OSM tags.
    /// Uses `bridge`, `bridge:material`, and `bridge:structure` tags.
    pub fn from_tags(tags: &HashMap<String, String>) -> Self {
        let bridge_type = tags.get("bridge").map(|s| s.as_str()).unwrap_or("yes");
        let material = tags.get("bridge:material").map(|s| s.as_str());
        let structure = tags.get("bridge:structure").map(|s| s.as_str());

        // Base config from bridge type
        let mut config = match bridge_type {
            "aqueduct" => Self {
                deck_block: STONE_BRICKS,
                railing_block: STONE_BRICK_WALL,
                support_block: STONE_BRICKS,
                has_railings: true,
                support_interval: 6,
                half_width: 2,
                structure_type: BridgeStructure::Arch,
            },
            "boardwalk" => Self {
                deck_block: OAK_PLANKS,
                railing_block: OAK_FENCE,
                support_block: OAK_LOG,
                has_railings: true,
                support_interval: 4,
                half_width: 1,
                structure_type: BridgeStructure::Beam,
            },
            "viaduct" => Self {
                deck_block: STONE_BRICKS,
                railing_block: STONE_BRICK_WALL,
                support_block: STONE_BRICKS,
                has_railings: true,
                support_interval: 6,
                half_width: 3,
                structure_type: BridgeStructure::Arch,
            },
            "covered" => Self {
                deck_block: OAK_PLANKS,
                railing_block: OAK_PLANKS,
                support_block: OAK_LOG,
                has_railings: true,
                support_interval: 5,
                half_width: 2,
                structure_type: BridgeStructure::Beam,
            },
            _ => Self {
                // Default for "yes" and any other unrecognised value
                deck_block: LIGHT_GRAY_CONCRETE,
                railing_block: COBBLESTONE_WALL,
                support_block: STONE_BRICKS,
                has_railings: true,
                support_interval: 8,
                half_width: 2,
                structure_type: BridgeStructure::Beam,
            },
        };

        // Override blocks based on bridge:material tag
        if let Some(mat) = material {
            match mat {
                "steel" | "metal" => {
                    config.deck_block = GRAY_CONCRETE;
                    config.railing_block = IRON_BARS;
                    config.support_block = IRON_BLOCK;
                }
                "stone" | "masonry" => {
                    config.deck_block = STONE_BRICKS;
                    config.railing_block = STONE_BRICK_WALL;
                    config.support_block = STONE_BRICKS;
                }
                "wood" | "timber" => {
                    config.deck_block = OAK_PLANKS;
                    config.railing_block = OAK_FENCE;
                    config.support_block = OAK_LOG;
                }
                "concrete" | "prestressed_concrete" | "reinforced_concrete" => {
                    config.deck_block = LIGHT_GRAY_CONCRETE;
                    config.railing_block = GRAY_CONCRETE;
                    config.support_block = GRAY_CONCRETE;
                }
                "brick" => {
                    config.deck_block = BRICK;
                    config.railing_block = BRICK_WALL;
                    config.support_block = BRICK;
                }
                _ => {}
            }
        }

        // Override structure type from bridge:structure tag
        if let Some(st) = structure {
            config.structure_type = match st {
                "arch" => BridgeStructure::Arch,
                "truss" => BridgeStructure::Truss,
                "suspension" => BridgeStructure::Suspension,
                "cable-stayed" | "cable_stayed" => BridgeStructure::CableStayed,
                "beam" | "simple-beam" => BridgeStructure::Beam,
                _ => config.structure_type, // Keep type-based default
            };
        }

        config
    }
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Calculate a flat bridge deck Y level from the way's node ground elevations.
/// Samples ALL nodes (not just endpoints) so the deck clears terrain everywhere.
pub fn calculate_bridge_deck_y(editor: &WorldEditor, nodes: &[ProcessedNode]) -> Option<i32> {
    if nodes.len() < 2 {
        return None;
    }
    nodes
        .iter()
        .map(|n| editor.get_ground_level(n.x, n.z))
        .max()
}

/// Place railings along both sides of a bridge segment.
pub fn place_railings_for_segment(
    editor: &mut WorldEditor,
    points: &[(i32, i32, i32)],
    deck_y: i32,
    config: &BridgeConfig,
    dir_x: f64,
    dir_z: f64,
) {
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

// ---------------------------------------------------------------------------
// Structural detail functions
// ---------------------------------------------------------------------------

/// Helper: compute segment direction and perpendicular vectors.
fn segment_vectors(prev: &ProcessedNode, cur: &ProcessedNode) -> (f64, f64, f64, f64) {
    let seg_dx = (cur.x - prev.x) as f64;
    let seg_dz = (cur.z - prev.z) as f64;
    let seg_len = (seg_dx * seg_dx + seg_dz * seg_dz).sqrt();
    let (dir_x, dir_z) = if seg_len > 0.0 {
        (seg_dx / seg_len, seg_dz / seg_len)
    } else {
        (1.0, 0.0)
    };
    let perp_x = -dir_z;
    let perp_z = dir_x;
    (dir_x, dir_z, perp_x, perp_z)
}

/// Place an arch understructure between support pillars.
/// Creates a parabolic arch curve underneath the bridge deck.
fn place_arch_understructure(
    editor: &mut WorldEditor,
    element: &ProcessedWay,
    deck_y: i32,
    config: &BridgeConfig,
) {
    let span = config.support_interval;
    if span < 4 {
        return; // Too short for a visible arch
    }
    let arch_depth = ((span as f64 / 3.0).round() as i32).clamp(2, 6);

    let mut block_counter: i32 = 0;

    for i in 1..element.nodes.len() {
        let prev = &element.nodes[i - 1];
        let cur = &element.nodes[i];
        let points = bresenham_line(prev.x, 0, prev.z, cur.x, 0, cur.z);
        let (_dir_x, _dir_z, perp_x, perp_z) = segment_vectors(prev, cur);

        for (x, _, z) in &points {
            let pos_in_span = block_counter % span;

            // Place arch between pillar positions
            if pos_in_span > 0 && pos_in_span < span {
                let t = 2.0 * (pos_in_span as f64) / (span as f64) - 1.0;
                let curve_offset = ((arch_depth as f64) * (1.0 - t * t)).round() as i32;
                let arch_y = deck_y - 2 - curve_offset;

                // Arch blocks span the full bridge width
                for w in -config.half_width..=config.half_width {
                    let ax = (*x as f64 + perp_x * w as f64).round() as i32;
                    let az = (*z as f64 + perp_z * w as f64).round() as i32;
                    let ground_y = editor.get_ground_level(ax, az);
                    if arch_y > ground_y {
                        editor.set_block_absolute(
                            config.support_block,
                            ax,
                            arch_y,
                            az,
                            None,
                            None,
                        );
                        // One block of thickness above the arch curve
                        if arch_y + 1 < deck_y - 1 {
                            editor.set_block_absolute(
                                config.support_block,
                                ax,
                                arch_y + 1,
                                az,
                                None,
                                None,
                            );
                        }
                    }
                }
            }

            block_counter += 1;
        }
    }
}

/// Place truss side bracing along the bridge.
/// Creates vertical posts with a top chord and diagonal cross-bracing.
fn place_truss_sides(
    editor: &mut WorldEditor,
    element: &ProcessedWay,
    deck_y: i32,
    config: &BridgeConfig,
) {
    const TRUSS_HEIGHT: i32 = 3;
    const POST_INTERVAL: i32 = 4;

    let mut block_counter: i32 = 0;

    for i in 1..element.nodes.len() {
        let prev = &element.nodes[i - 1];
        let cur = &element.nodes[i];
        let points = bresenham_line(prev.x, 0, prev.z, cur.x, 0, cur.z);
        let (_dir_x, _dir_z, perp_x, perp_z) = segment_vectors(prev, cur);

        for (x, _, z) in &points {
            for &side in &[-1.0_f64, 1.0_f64] {
                let offset = (config.half_width as f64 + 1.0) * side;
                let rx = (*x as f64 + perp_x * offset).round() as i32;
                let rz = (*z as f64 + perp_z * offset).round() as i32;

                if block_counter % POST_INTERVAL == 0 {
                    // Vertical post from deck+1 to deck+TRUSS_HEIGHT
                    for dy in 1..=TRUSS_HEIGHT {
                        editor.set_block_absolute(
                            config.support_block,
                            rx,
                            deck_y + dy,
                            rz,
                            None,
                            None,
                        );
                    }
                } else {
                    // Top chord beam
                    editor.set_block_absolute(
                        config.support_block,
                        rx,
                        deck_y + TRUSS_HEIGHT,
                        rz,
                        None,
                        None,
                    );
                    // Diagonal cross-bracing at mid-height
                    if block_counter % 2 == 0 {
                        editor.set_block_absolute(
                            IRON_BARS,
                            rx,
                            deck_y + 2,
                            rz,
                            None,
                            None,
                        );
                    }
                }
            }

            block_counter += 1;
        }
    }
}

/// Place suspension bridge towers and draped main cables with vertical suspenders.
fn place_suspension_cables(
    editor: &mut WorldEditor,
    element: &ProcessedWay,
    deck_y: i32,
    config: &BridgeConfig,
) {
    const TOWER_HEIGHT: i32 = 8;
    let tower_interval = (config.support_interval * 3).max(12);

    let mut block_counter: i32 = 0;

    for i in 1..element.nodes.len() {
        let prev = &element.nodes[i - 1];
        let cur = &element.nodes[i];
        let points = bresenham_line(prev.x, 0, prev.z, cur.x, 0, cur.z);
        let (_dir_x, _dir_z, perp_x, perp_z) = segment_vectors(prev, cur);

        for (x, _, z) in &points {
            let is_tower = block_counter > 0 && block_counter % tower_interval == 0;

            // Tower pylons
            if is_tower {
                for &side in &[-1.0_f64, 1.0_f64] {
                    let offset = (config.half_width as f64 + 1.0) * side;
                    let rx = (*x as f64 + perp_x * offset).round() as i32;
                    let rz = (*z as f64 + perp_z * offset).round() as i32;
                    for dy in 1..=TOWER_HEIGHT {
                        editor.set_block_absolute(
                            config.support_block,
                            rx,
                            deck_y + dy,
                            rz,
                            None,
                            None,
                        );
                    }
                }
            }

            // Main cable (parabolic catenary between towers)
            let pos_in_span = block_counter % tower_interval;
            if pos_in_span > 0 {
                let t = 2.0 * (pos_in_span as f64) / (tower_interval as f64) - 1.0;
                let sag = ((TOWER_HEIGHT as f64 - 1.0) * t * t).round() as i32;
                let cable_y = deck_y + TOWER_HEIGHT - sag;

                for &side in &[-1.0_f64, 1.0_f64] {
                    let offset = (config.half_width as f64 + 1.0) * side;
                    let rx = (*x as f64 + perp_x * offset).round() as i32;
                    let rz = (*z as f64 + perp_z * offset).round() as i32;
                    editor.set_block_absolute(CHAIN, rx, cable_y, rz, None, None);
                }

                // Vertical suspender cables every 3 blocks
                if pos_in_span % 3 == 0 {
                    for &side in &[-1.0_f64, 1.0_f64] {
                        let offset = (config.half_width as f64 + 1.0) * side;
                        let rx = (*x as f64 + perp_x * offset).round() as i32;
                        let rz = (*z as f64 + perp_z * offset).round() as i32;
                        for y in (deck_y + 2)..cable_y {
                            editor.set_block_absolute(CHAIN, rx, y, rz, None, None);
                        }
                    }
                }
            }

            block_counter += 1;
        }
    }
}

/// Place cable-stayed bridge pylons with fan-shaped diagonal cables.
fn place_cable_stayed_structure(
    editor: &mut WorldEditor,
    element: &ProcessedWay,
    deck_y: i32,
    config: &BridgeConfig,
) {
    const PYLON_HEIGHT: i32 = 10;
    let pylon_interval = (config.support_interval * 4).max(16);

    // Collect all bridge points with their perpendicular vectors
    let mut all_points: Vec<(i32, i32, f64, f64)> = Vec::new();

    for i in 1..element.nodes.len() {
        let prev = &element.nodes[i - 1];
        let cur = &element.nodes[i];
        let points = bresenham_line(prev.x, 0, prev.z, cur.x, 0, cur.z);
        let (_dir_x, _dir_z, perp_x, perp_z) = segment_vectors(prev, cur);

        for (x, _, z) in &points {
            all_points.push((*x, *z, perp_x, perp_z));
        }
    }

    let total = all_points.len() as i32;

    for (idx, (x, z, perp_x, perp_z)) in all_points.iter().enumerate() {
        let block_counter = idx as i32;
        let is_pylon =
            block_counter > 0 && block_counter < total - 1 && block_counter % pylon_interval == 0;

        if is_pylon {
            // Pylon towers on both sides
            for &side in &[-1.0_f64, 1.0_f64] {
                let offset = (config.half_width as f64 + 1.0) * side;
                let rx = (*x as f64 + perp_x * offset).round() as i32;
                let rz = (*z as f64 + perp_z * offset).round() as i32;
                for dy in 1..=PYLON_HEIGHT {
                    editor.set_block_absolute(
                        config.support_block,
                        rx,
                        deck_y + dy,
                        rz,
                        None,
                        None,
                    );
                }
            }

            // Diagonal cables radiating from pylon top to deck
            let cable_reach = (pylon_interval / 2).min(total - block_counter).min(block_counter);
            for dist in (2..cable_reach).step_by(2) {
                let cable_y =
                    deck_y + PYLON_HEIGHT - (PYLON_HEIGHT * dist / cable_reach).max(1);

                // Forward cable
                let fwd = (idx as i32 + dist).min(total - 1) as usize;
                if fwd < all_points.len() {
                    let (fx, fz, fp_x, fp_z) = all_points[fwd];
                    for &side in &[-1.0_f64, 1.0_f64] {
                        let offset = (config.half_width as f64 + 1.0) * side;
                        let rx = (fx as f64 + fp_x * offset).round() as i32;
                        let rz = (fz as f64 + fp_z * offset).round() as i32;
                        editor.set_block_absolute(CHAIN, rx, cable_y, rz, None, None);
                    }
                }

                // Backward cable
                let bwd = (idx as i32 - dist).max(0) as usize;
                if bwd < all_points.len() {
                    let (bx, bz, bp_x, bp_z) = all_points[bwd];
                    for &side in &[-1.0_f64, 1.0_f64] {
                        let offset = (config.half_width as f64 + 1.0) * side;
                        let rx = (bx as f64 + bp_x * offset).round() as i32;
                        let rz = (bz as f64 + bp_z * offset).round() as i32;
                        editor.set_block_absolute(CHAIN, rx, cable_y, rz, None, None);
                    }
                }
            }
        }
    }
}

/// Place bridge abutments (retaining walls) at the start and end of a bridge.
pub fn place_bridge_abutments(
    editor: &mut WorldEditor,
    element: &ProcessedWay,
    deck_y: i32,
    config: &BridgeConfig,
) {
    if element.nodes.len() < 2 {
        return;
    }

    let endpoints = [
        (&element.nodes[0], &element.nodes[1]),
        (
            &element.nodes[element.nodes.len() - 1],
            &element.nodes[element.nodes.len() - 2],
        ),
    ];

    for (end_node, adjacent_node) in &endpoints {
        let ground_y = editor.get_ground_level(end_node.x, end_node.z);
        if deck_y <= ground_y + 1 {
            continue; // No abutment needed at ground level
        }

        let dx = (end_node.x - adjacent_node.x) as f64;
        let dz = (end_node.z - adjacent_node.z) as f64;
        let len = (dx * dx + dz * dz).sqrt();
        let (_dir_x, _dir_z, perp_x, perp_z) = if len > 0.0 {
            let dir_x = dx / len;
            let dir_z = dz / len;
            (dir_x, dir_z, -dir_z, dir_x)
        } else {
            (1.0, 0.0, 0.0, 1.0)
        };

        // Abutment wall perpendicular to bridge direction
        let abutment_width = config.half_width + 1;
        for w in -abutment_width..=abutment_width {
            let ax = (end_node.x as f64 + perp_x * w as f64).round() as i32;
            let az = (end_node.z as f64 + perp_z * w as f64).round() as i32;
            let local_ground = editor.get_ground_level(ax, az);
            for y in local_ground..=deck_y {
                editor.set_block_absolute(config.support_block, ax, y, az, None, None);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Public API functions
// ---------------------------------------------------------------------------

/// Generates a standalone bridge for ways that have a `bridge=*` tag but no
/// primary infrastructure tag (highway, railway, etc.).
pub fn generate_bridges(editor: &mut WorldEditor, element: &ProcessedWay) {
    let _bridge_type = match element.tags.get("bridge") {
        Some(bt) if bt != "no" => bt.clone(),
        _ => return,
    };

    if element.nodes.len() < 2 {
        return;
    }

    let config = BridgeConfig::from_tags(&element.tags);

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
        let (dir_x, dir_z, _perp_x, _perp_z) = segment_vectors(prev, cur);

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

    // Structural details based on bridge:structure type
    match config.structure_type {
        BridgeStructure::Arch => place_arch_understructure(editor, element, deck_y, &config),
        BridgeStructure::Truss => place_truss_sides(editor, element, deck_y, &config),
        BridgeStructure::Suspension => place_suspension_cables(editor, element, deck_y, &config),
        BridgeStructure::CableStayed => {
            place_cable_stayed_structure(editor, element, deck_y, &config)
        }
        BridgeStructure::Beam => {}
    }

    // Abutments at bridge endpoints
    place_bridge_abutments(editor, element, deck_y, &config);
}

/// Adds railings to an existing highway bridge segment.
///
/// Called from `highways.rs` for ways that have `bridge=yes` and are elevated.
/// `point_elevations` holds the Y value for each point (absolute when
/// `elevations_are_absolute` is true, ground-relative otherwise).
/// `block_range` is the highway half-width, and `railing_block` the block to use.
pub fn add_highway_bridge_railings(
    editor: &mut WorldEditor,
    points: &[(i32, i32, i32)],
    point_elevations: &[i32],
    elevations_are_absolute: bool,
    block_range: i32,
    railing_block: Block,
    dir_x: f64,
    dir_z: f64,
) {
    let perp_x = -dir_z;
    let perp_z = dir_x;

    for (i, (x, _, z)) in points.iter().enumerate() {
        let y = point_elevations.get(i).copied().unwrap_or(0);
        for &side in &[-1.0_f64, 1.0_f64] {
            let offset = (block_range as f64 + 1.0) * side;
            let rx = (*x as f64 + perp_x * offset).round() as i32;
            let rz = (*z as f64 + perp_z * offset).round() as i32;

            let railing_y = if elevations_are_absolute {
                y + 1
            } else {
                editor.get_absolute_y(rx, y + 1, rz)
            };

            editor.set_block_absolute(
                railing_block,
                rx,
                railing_y,
                rz,
                None,
                Some(&[railing_block]),
            );
        }
    }
}

/// Adds railings to a railway bridge at a single block position.
///
/// Called per-block from `railways.rs` for railway ways with `bridge=*`.
/// `deck_y` is the absolute Y of the gravel bed.
pub fn add_railway_bridge_railings(
    editor: &mut WorldEditor,
    bx: i32,
    bz: i32,
    deck_y: i32,
    config: &BridgeConfig,
    dir_x: f64,
    dir_z: f64,
) {
    let perp_x = -dir_z;
    let perp_z = dir_x;

    for &side in &[-1.0_f64, 1.0_f64] {
        let offset = 2.0 * side; // Rail gauge offset
        let rx = (bx as f64 + perp_x * offset).round() as i32;
        let rz = (bz as f64 + perp_z * offset).round() as i32;
        // Railing at deck+2 (one above the rail at deck+1)
        editor.set_block_absolute(config.railing_block, rx, deck_y + 2, rz, None, None);
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

    let config = BridgeConfig::from_tags(&element.tags);
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

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

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
        assert_eq!(config.structure_type, BridgeStructure::Beam);
    }

    #[test]
    fn test_config_aqueduct() {
        let config = BridgeConfig::from_bridge_type("aqueduct");
        assert_eq!(config.deck_block, STONE_BRICKS);
        assert_eq!(config.railing_block, STONE_BRICK_WALL);
        assert!(config.has_railings);
        assert_eq!(config.structure_type, BridgeStructure::Arch);
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
        assert_eq!(config.structure_type, BridgeStructure::Arch);
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

    #[test]
    fn test_config_from_tags_material_steel() {
        let mut tags = HashMap::new();
        tags.insert("bridge".to_string(), "yes".to_string());
        tags.insert("bridge:material".to_string(), "steel".to_string());
        let config = BridgeConfig::from_tags(&tags);
        assert_eq!(config.deck_block, GRAY_CONCRETE);
        assert_eq!(config.railing_block, IRON_BARS);
        assert_eq!(config.support_block, IRON_BLOCK);
    }

    #[test]
    fn test_config_from_tags_material_wood() {
        let mut tags = HashMap::new();
        tags.insert("bridge".to_string(), "yes".to_string());
        tags.insert("bridge:material".to_string(), "wood".to_string());
        let config = BridgeConfig::from_tags(&tags);
        assert_eq!(config.deck_block, OAK_PLANKS);
        assert_eq!(config.railing_block, OAK_FENCE);
        assert_eq!(config.support_block, OAK_LOG);
    }

    #[test]
    fn test_config_from_tags_material_concrete() {
        let mut tags = HashMap::new();
        tags.insert("bridge".to_string(), "yes".to_string());
        tags.insert("bridge:material".to_string(), "concrete".to_string());
        let config = BridgeConfig::from_tags(&tags);
        assert_eq!(config.deck_block, LIGHT_GRAY_CONCRETE);
        assert_eq!(config.railing_block, GRAY_CONCRETE);
        assert_eq!(config.support_block, GRAY_CONCRETE);
    }

    #[test]
    fn test_config_from_tags_material_brick() {
        let mut tags = HashMap::new();
        tags.insert("bridge".to_string(), "yes".to_string());
        tags.insert("bridge:material".to_string(), "brick".to_string());
        let config = BridgeConfig::from_tags(&tags);
        assert_eq!(config.deck_block, BRICK);
        assert_eq!(config.railing_block, BRICK_WALL);
        assert_eq!(config.support_block, BRICK);
    }

    #[test]
    fn test_config_from_tags_structure_override() {
        let mut tags = HashMap::new();
        tags.insert("bridge".to_string(), "yes".to_string());
        tags.insert("bridge:structure".to_string(), "truss".to_string());
        let config = BridgeConfig::from_tags(&tags);
        assert_eq!(config.structure_type, BridgeStructure::Truss);
    }

    #[test]
    fn test_config_from_tags_structure_suspension() {
        let mut tags = HashMap::new();
        tags.insert("bridge".to_string(), "yes".to_string());
        tags.insert("bridge:structure".to_string(), "suspension".to_string());
        let config = BridgeConfig::from_tags(&tags);
        assert_eq!(config.structure_type, BridgeStructure::Suspension);
    }

    #[test]
    fn test_config_from_tags_structure_cable_stayed() {
        let mut tags = HashMap::new();
        tags.insert("bridge".to_string(), "yes".to_string());
        tags.insert("bridge:structure".to_string(), "cable-stayed".to_string());
        let config = BridgeConfig::from_tags(&tags);
        assert_eq!(config.structure_type, BridgeStructure::CableStayed);
    }

    #[test]
    fn test_config_from_tags_material_and_structure() {
        let mut tags = HashMap::new();
        tags.insert("bridge".to_string(), "yes".to_string());
        tags.insert("bridge:material".to_string(), "steel".to_string());
        tags.insert("bridge:structure".to_string(), "arch".to_string());
        let config = BridgeConfig::from_tags(&tags);
        assert_eq!(config.deck_block, GRAY_CONCRETE);
        assert_eq!(config.railing_block, IRON_BARS);
        assert_eq!(config.structure_type, BridgeStructure::Arch);
    }

    #[test]
    fn test_config_from_tags_unknown_material_keeps_defaults() {
        let mut tags = HashMap::new();
        tags.insert("bridge".to_string(), "yes".to_string());
        tags.insert("bridge:material".to_string(), "carbon_fiber".to_string());
        let config = BridgeConfig::from_tags(&tags);
        assert_eq!(config.deck_block, LIGHT_GRAY_CONCRETE);
        assert_eq!(config.railing_block, COBBLESTONE_WALL);
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
    fn test_deck_y_returns_max_of_all_nodes() {
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
                x: 30,
                z: 30,
            },
            ProcessedNode {
                id: 2,
                tags: HashMap::new(),
                x: 50,
                z: 50,
            },
        ];
        let y = calculate_bridge_deck_y(&editor, &nodes);
        assert_eq!(y, Some(0)); // All at ground level 0 without terrain
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
        generate_bridges(&mut editor, &way);
        assert!(!editor.block_at_absolute(30, 3, 10));
    }

    #[test]
    fn test_generate_bridges_single_node_skips() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let mut editor = test_editor(&xzbbox);
        let way = make_bridge_way("yes", vec![(10, 10)], vec![]);
        generate_bridges(&mut editor, &way);
        assert!(!editor.block_at_absolute(10, 3, 10));
    }

    #[test]
    fn test_generate_bridges_places_deck() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let mut editor = test_editor(&xzbbox);
        let way = make_bridge_way("yes", vec![(10, 50), (50, 50)], vec![]);
        generate_bridges(&mut editor, &way);
        // Default layer=1 -> bridge_height=3, deck_y=0+3=3
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
        assert!(editor.block_at_absolute(30, 3, 50)); // Centre point
        assert!(editor.block_at_absolute(9, 3, 50)); // half_width=1
        assert!(!editor.block_at_absolute(8, 3, 50)); // outside half_width
    }

    #[test]
    fn test_generate_bridges_with_material_tag() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let mut editor = test_editor(&xzbbox);
        let way = make_bridge_way(
            "yes",
            vec![(10, 50), (50, 50)],
            vec![("bridge:material", "steel")],
        );
        generate_bridges(&mut editor, &way);
        // Deck placed at y=3 (layer=1 default)
        assert!(editor.block_at_absolute(30, 3, 50));
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
        let point_elevations: Vec<i32> = vec![deck_y; points.len()];
        let block_range = 2;
        // Direction is along x-axis
        add_highway_bridge_railings(
            &mut editor,
            &points,
            &point_elevations,
            true, // absolute Y
            block_range,
            COBBLESTONE_WALL,
            1.0,
            0.0,
        );
        // Railings should be at z=50 +/- (block_range+1) = z=47 and z=53
        // at y = deck_y + 1 = 6
        assert!(editor.block_at_absolute(30, 6, 47));
        assert!(editor.block_at_absolute(30, 6, 53));
        // No railing in the centre
        assert!(!editor.block_at_absolute(30, 6, 50));
    }

    #[test]
    fn test_highway_bridge_railings_per_point_elevation() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let mut editor = test_editor(&xzbbox);
        let points: Vec<(i32, i32, i32)> = (20..=22).map(|x| (x, 0, 50)).collect();
        // Each point has a different absolute Y
        let point_elevations = vec![3, 5, 7];
        add_highway_bridge_railings(
            &mut editor,
            &points,
            &point_elevations,
            true,
            2,
            COBBLESTONE_WALL,
            1.0,
            0.0,
        );
        // First point railing at y=4, second at y=6, third at y=8
        assert!(editor.block_at_absolute(20, 4, 47));
        assert!(editor.block_at_absolute(21, 6, 47));
        assert!(editor.block_at_absolute(22, 8, 47));
    }

    #[test]
    fn test_highway_bridge_railings_custom_block() {
        let xzbbox = XZBBox::rect_from_xz_lengths(100.0, 100.0).unwrap();
        let mut editor = test_editor(&xzbbox);
        let points: Vec<(i32, i32, i32)> = vec![(30, 0, 50)];
        let point_elevations = vec![5];
        add_highway_bridge_railings(
            &mut editor,
            &points,
            &point_elevations,
            true,
            2,
            IRON_BARS,
            1.0,
            0.0,
        );
        // Railing placed at y=6
        assert!(editor.block_at_absolute(30, 6, 47));
    }
}

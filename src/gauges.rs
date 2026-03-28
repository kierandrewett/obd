use egui::{self, Align2, Color32, FontId, Pos2, Stroke, Vec2};

/// Theme-adaptive colors derived from egui visuals
struct ThemeColors {
    gauge_bg: Color32,
    gauge_border: Color32,
    tick_major: Color32,
    tick_mid: Color32,
    tick_minor: Color32,
    tick_label: Color32,
    needle: Color32,
    needle_shadow: Color32,
    center_cap: Color32,
    value_text: Color32,
    label_text: Color32,
    bar_bg: Color32,
    bar_label: Color32,
    bar_value: Color32,
    spark_bg: Color32,
    spark_border: Color32,
}

impl ThemeColors {
    fn from_ui(ui: &egui::Ui) -> Self {
        if ui.visuals().dark_mode {
            Self {
                gauge_bg: Color32::from_gray(22),
                gauge_border: Color32::from_gray(50),
                tick_major: Color32::from_gray(180),
                tick_mid: Color32::from_gray(100),
                tick_minor: Color32::from_gray(60),
                tick_label: Color32::from_gray(140),
                needle: Color32::from_gray(230),
                needle_shadow: Color32::from_black_alpha(80),
                center_cap: Color32::from_gray(50),
                value_text: Color32::WHITE,
                label_text: Color32::from_gray(120),
                bar_bg: Color32::from_gray(40),
                bar_label: Color32::from_gray(180),
                bar_value: Color32::WHITE,
                spark_bg: Color32::from_gray(20),
                spark_border: Color32::from_gray(40),
            }
        } else {
            Self {
                gauge_bg: Color32::from_gray(240),
                gauge_border: Color32::from_gray(190),
                tick_major: Color32::from_gray(60),
                tick_mid: Color32::from_gray(140),
                tick_minor: Color32::from_gray(190),
                tick_label: Color32::from_gray(80),
                needle: Color32::from_gray(30),
                needle_shadow: Color32::from_black_alpha(30),
                center_cap: Color32::from_gray(200),
                value_text: Color32::from_gray(20),
                label_text: Color32::from_gray(100),
                bar_bg: Color32::from_gray(215),
                bar_label: Color32::from_gray(60),
                bar_value: Color32::from_gray(20),
                spark_bg: Color32::from_gray(235),
                spark_border: Color32::from_gray(200),
            }
        }
    }
}

/// A radial gauge widget (speedometer/tachometer style)
pub struct RadialGauge<'a> {
    pub label: &'a str,
    pub value: f64,
    pub min: f64,
    pub max: f64,
    pub unit: &'a str,
    pub size: f32,
    pub warning_threshold: Option<f64>,
    pub danger_threshold: Option<f64>,
    pub decimals: usize,
}

impl<'a> RadialGauge<'a> {
    pub fn new(label: &'a str, value: f64, min: f64, max: f64, unit: &'a str) -> Self {
        Self {
            label,
            value,
            min,
            max,
            unit,
            size: 140.0,
            warning_threshold: None,
            danger_threshold: None,
            decimals: 0,
        }
    }

    pub fn size(mut self, size: f32) -> Self {
        self.size = size;
        self
    }

    pub fn warning(mut self, threshold: f64) -> Self {
        self.warning_threshold = Some(threshold);
        self
    }

    pub fn danger(mut self, threshold: f64) -> Self {
        self.danger_threshold = Some(threshold);
        self
    }

    pub fn decimals(mut self, d: usize) -> Self {
        self.decimals = d;
        self
    }

    pub fn show(self, ui: &mut egui::Ui) -> egui::Response {
        let desired_size = Vec2::splat(self.size);
        let (rect, response) = ui.allocate_exact_size(desired_size, egui::Sense::hover());

        if !ui.is_rect_visible(rect) {
            return response;
        }

        let tc = ThemeColors::from_ui(ui);
        let painter = ui.painter_at(rect);
        let center = rect.center();
        let radius = self.size * 0.42;

        // Background circle
        painter.circle_filled(center, radius + 3.0, tc.gauge_bg);
        painter.circle_stroke(center, radius + 3.0, Stroke::new(1.5, tc.gauge_border));

        // Arc parameters: sweep from 225 degrees to -45 degrees (270 degree arc)
        let start_angle = 225.0_f32.to_radians();
        let end_angle = -45.0_f32.to_radians();
        let total_sweep = start_angle - end_angle;

        // ── Colored arc band (thick outer ring showing thresholds) ──────
        let arc_segments = 120;
        let arc_outer = radius * 0.97;
        let arc_inner = radius * 0.88;
        for i in 0..arc_segments {
            let frac0 = i as f32 / arc_segments as f32;
            let frac1 = (i + 1) as f32 / arc_segments as f32;
            let a0 = start_angle - frac0 * total_sweep;
            let a1 = start_angle - frac1 * total_sweep;
            let tick_value = self.min + frac0 as f64 * (self.max - self.min);

            let base_color = self.threshold_color(tick_value);
            // Dim the arc band
            let arc_color = Color32::from_rgba_premultiplied(
                base_color.r() / 3,
                base_color.g() / 3,
                base_color.b() / 3,
                200,
            );

            // Draw a small quad as two triangles
            let p0_out = Pos2::new(
                center.x + arc_outer * a0.cos(),
                center.y - arc_outer * a0.sin(),
            );
            let p1_out = Pos2::new(
                center.x + arc_outer * a1.cos(),
                center.y - arc_outer * a1.sin(),
            );
            let p0_in = Pos2::new(
                center.x + arc_inner * a0.cos(),
                center.y - arc_inner * a0.sin(),
            );
            let p1_in = Pos2::new(
                center.x + arc_inner * a1.cos(),
                center.y - arc_inner * a1.sin(),
            );

            let mesh = egui::Mesh {
                vertices: vec![
                    egui::epaint::Vertex {
                        pos: p0_out,
                        uv: egui::epaint::WHITE_UV,
                        color: arc_color,
                    },
                    egui::epaint::Vertex {
                        pos: p1_out,
                        uv: egui::epaint::WHITE_UV,
                        color: arc_color,
                    },
                    egui::epaint::Vertex {
                        pos: p1_in,
                        uv: egui::epaint::WHITE_UV,
                        color: arc_color,
                    },
                    egui::epaint::Vertex {
                        pos: p0_in,
                        uv: egui::epaint::WHITE_UV,
                        color: arc_color,
                    },
                ],
                indices: vec![0, 1, 2, 0, 2, 3],
                texture_id: egui::TextureId::default(),
            };
            painter.add(egui::Shape::mesh(mesh));
        }

        // ── Bright arc up to current value ──────────────────────────────
        let value_frac =
            ((self.value.clamp(self.min, self.max) - self.min) / (self.max - self.min)) as f32;
        let value_segments = (value_frac * arc_segments as f32) as usize;
        let bright_outer = radius * 0.97;
        let bright_inner = radius * 0.90;
        for i in 0..value_segments {
            let frac0 = i as f32 / arc_segments as f32;
            let frac1 = (i + 1) as f32 / arc_segments as f32;
            let a0 = start_angle - frac0 * total_sweep;
            let a1 = start_angle - frac1 * total_sweep;
            let tick_value = self.min + frac0 as f64 * (self.max - self.min);
            let color = self.threshold_color(tick_value);

            let p0_out = Pos2::new(
                center.x + bright_outer * a0.cos(),
                center.y - bright_outer * a0.sin(),
            );
            let p1_out = Pos2::new(
                center.x + bright_outer * a1.cos(),
                center.y - bright_outer * a1.sin(),
            );
            let p0_in = Pos2::new(
                center.x + bright_inner * a0.cos(),
                center.y - bright_inner * a0.sin(),
            );
            let p1_in = Pos2::new(
                center.x + bright_inner * a1.cos(),
                center.y - bright_inner * a1.sin(),
            );

            let mesh = egui::Mesh {
                vertices: vec![
                    egui::epaint::Vertex {
                        pos: p0_out,
                        uv: egui::epaint::WHITE_UV,
                        color,
                    },
                    egui::epaint::Vertex {
                        pos: p1_out,
                        uv: egui::epaint::WHITE_UV,
                        color,
                    },
                    egui::epaint::Vertex {
                        pos: p1_in,
                        uv: egui::epaint::WHITE_UV,
                        color,
                    },
                    egui::epaint::Vertex {
                        pos: p0_in,
                        uv: egui::epaint::WHITE_UV,
                        color,
                    },
                ],
                indices: vec![0, 1, 2, 0, 2, 3],
                texture_id: egui::TextureId::default(),
            };
            painter.add(egui::Shape::mesh(mesh));
        }

        // ── Tick marks ──────────────────────────────────────────────────
        let tick_count = 40;
        for i in 0..=tick_count {
            let frac = i as f32 / tick_count as f32;
            let angle = start_angle - frac * total_sweep;
            let is_major = i % 10 == 0;
            let is_mid = i % 5 == 0;

            let inner = if is_major {
                radius * 0.78
            } else {
                radius * 0.84
            };
            let outer = radius * 0.87;
            let cos = angle.cos();
            let sin = angle.sin();
            let p1 = Pos2::new(center.x + inner * cos, center.y - inner * sin);
            let p2 = Pos2::new(center.x + outer * cos, center.y - outer * sin);

            let tick_color = if is_major {
                tc.tick_major
            } else if is_mid {
                tc.tick_mid
            } else {
                tc.tick_minor
            };
            let width = if is_major { 2.0 } else { 1.0 };
            painter.line_segment([p1, p2], Stroke::new(width, tick_color));

            // Major tick labels
            if is_major {
                let label_r = radius * 0.67;
                let label_pos = Pos2::new(center.x + label_r * cos, center.y - label_r * sin);
                let label_val = self.min + frac as f64 * (self.max - self.min);
                let label_text = if self.max >= 1000.0 {
                    format!("{}", label_val as i64)
                } else {
                    format!("{label_val:.0}")
                };
                painter.text(
                    label_pos,
                    Align2::CENTER_CENTER,
                    label_text,
                    FontId::proportional(self.size * 0.065),
                    tc.tick_label,
                );
            }
        }

        // ── Needle ──────────────────────────────────────────────────────
        let clamped = self.value.clamp(self.min, self.max);
        let frac = (clamped - self.min) / (self.max - self.min);
        let needle_angle = start_angle - frac as f32 * total_sweep;
        let needle_len = radius * 0.72;
        let needle_tip = Pos2::new(
            center.x + needle_len * needle_angle.cos(),
            center.y - needle_len * needle_angle.sin(),
        );

        let needle_color = if let Some(danger) = self.danger_threshold {
            if self.value >= danger {
                Color32::from_rgb(255, 60, 60)
            } else {
                tc.needle
            }
        } else {
            tc.needle
        };

        // Needle shadow
        let shadow_tip = Pos2::new(needle_tip.x + 1.0, needle_tip.y + 1.0);
        painter.line_segment(
            [Pos2::new(center.x + 1.0, center.y + 1.0), shadow_tip],
            Stroke::new(3.0, tc.needle_shadow),
        );
        painter.line_segment([center, needle_tip], Stroke::new(2.0, needle_color));
        // Center cap
        painter.circle_filled(center, 5.0, tc.center_cap);
        painter.circle_filled(center, 3.0, needle_color);

        // ── Value text ──────────────────────────────────────────────────
        let value_text = match self.decimals {
            0 => format!("{:.0}", self.value),
            1 => format!("{:.1}", self.value),
            _ => format!("{:.2}", self.value),
        };
        painter.text(
            Pos2::new(center.x, center.y + radius * 0.38),
            Align2::CENTER_CENTER,
            format!("{} {}", value_text, self.unit),
            FontId::proportional(self.size * 0.11),
            tc.value_text,
        );

        // Label
        painter.text(
            Pos2::new(center.x, center.y + radius * 0.58),
            Align2::CENTER_CENTER,
            self.label,
            FontId::proportional(self.size * 0.07),
            tc.label_text,
        );

        response
    }

    fn threshold_color(&self, tick_value: f64) -> Color32 {
        if let Some(danger) = self.danger_threshold {
            if tick_value >= danger {
                return Color32::from_rgb(220, 50, 50);
            }
        }
        if let Some(warn) = self.warning_threshold {
            if tick_value >= warn {
                return Color32::from_rgb(220, 180, 50);
            }
        }
        if self.danger_threshold.is_some() || self.warning_threshold.is_some() {
            Color32::from_rgb(50, 180, 100)
        } else {
            Color32::from_rgb(60, 140, 210)
        }
    }
}

/// A horizontal bar gauge
pub struct BarGauge<'a> {
    pub label: &'a str,
    pub value: f64,
    pub min: f64,
    pub max: f64,
    pub unit: &'a str,
    pub width: f32,
    pub warning_threshold: Option<f64>,
    pub danger_threshold: Option<f64>,
    pub decimals: usize,
}

#[allow(dead_code)]
impl<'a> BarGauge<'a> {
    pub fn new(label: &'a str, value: f64, min: f64, max: f64, unit: &'a str) -> Self {
        Self {
            label,
            value,
            min,
            max,
            unit,
            width: 200.0,
            warning_threshold: None,
            danger_threshold: None,
            decimals: 1,
        }
    }

    pub fn width(mut self, w: f32) -> Self {
        self.width = w;
        self
    }

    pub fn warning(mut self, threshold: f64) -> Self {
        self.warning_threshold = Some(threshold);
        self
    }

    pub fn danger(mut self, threshold: f64) -> Self {
        self.danger_threshold = Some(threshold);
        self
    }

    pub fn decimals(mut self, d: usize) -> Self {
        self.decimals = d;
        self
    }

    pub fn show(self, ui: &mut egui::Ui) {
        let tc = ThemeColors::from_ui(ui);
        let bar_height = 20.0;

        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new(self.label)
                    .color(tc.bar_label)
                    .size(13.0),
            );

            let frac = ((self.value - self.min) / (self.max - self.min)).clamp(0.0, 1.0) as f32;

            let (rect, _) =
                ui.allocate_exact_size(Vec2::new(self.width, bar_height), egui::Sense::hover());
            let painter = ui.painter_at(rect);

            // Background
            painter.rect_filled(rect, 3.0, tc.bar_bg);

            // Fill
            let fill_color = if let Some(danger) = self.danger_threshold {
                if self.value >= danger {
                    Color32::from_rgb(220, 50, 50)
                } else if let Some(warn) = self.warning_threshold {
                    if self.value >= warn {
                        Color32::from_rgb(220, 180, 50)
                    } else {
                        Color32::from_rgb(50, 180, 100)
                    }
                } else {
                    Color32::from_rgb(50, 180, 100)
                }
            } else {
                Color32::from_rgb(80, 160, 220)
            };

            let fill_rect = egui::Rect::from_min_max(
                rect.min,
                Pos2::new(rect.min.x + rect.width() * frac, rect.max.y),
            );
            painter.rect_filled(fill_rect, 3.0, fill_color);

            // Value text
            let value_text = match self.decimals {
                0 => format!("{:.0} {}", self.value, self.unit),
                1 => format!("{:.1} {}", self.value, self.unit),
                _ => format!("{:.2} {}", self.value, self.unit),
            };
            ui.label(
                egui::RichText::new(value_text)
                    .color(tc.bar_value)
                    .size(13.0)
                    .strong(),
            );
        });
    }
}

/// Sparkline with gradient fill
pub fn sparkline(ui: &mut egui::Ui, history: &[f64], width: f32, height: f32, color: Color32) {
    if history.len() < 2 {
        return;
    }

    let tc = ThemeColors::from_ui(ui);
    let (rect, _) = ui.allocate_exact_size(Vec2::new(width, height), egui::Sense::hover());
    let painter = ui.painter_at(rect);

    // Background
    painter.rect_filled(rect, 3.0, tc.spark_bg);
    painter.rect_stroke(
        rect,
        3.0,
        Stroke::new(0.5, tc.spark_border),
        egui::StrokeKind::Outside,
    );

    let pad = 2.0;
    let inner_rect = rect.shrink(pad);

    let min_val = history.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_val = history.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let range = (max_val - min_val).max(0.001);

    let points: Vec<Pos2> = history
        .iter()
        .enumerate()
        .map(|(i, &v)| {
            let x = inner_rect.min.x + (i as f32 / (history.len() - 1) as f32) * inner_rect.width();
            let y = inner_rect.max.y - ((v - min_val) / range) as f32 * inner_rect.height();
            Pos2::new(x, y)
        })
        .collect();

    // Filled area under the line
    let fill_color =
        Color32::from_rgba_premultiplied(color.r() / 4, color.g() / 4, color.b() / 4, 60);
    for window in points.windows(2) {
        let mesh = egui::Mesh {
            vertices: vec![
                egui::epaint::Vertex {
                    pos: window[0],
                    uv: egui::epaint::WHITE_UV,
                    color: fill_color,
                },
                egui::epaint::Vertex {
                    pos: window[1],
                    uv: egui::epaint::WHITE_UV,
                    color: fill_color,
                },
                egui::epaint::Vertex {
                    pos: Pos2::new(window[1].x, inner_rect.max.y),
                    uv: egui::epaint::WHITE_UV,
                    color: Color32::TRANSPARENT,
                },
                egui::epaint::Vertex {
                    pos: Pos2::new(window[0].x, inner_rect.max.y),
                    uv: egui::epaint::WHITE_UV,
                    color: Color32::TRANSPARENT,
                },
            ],
            indices: vec![0, 1, 2, 0, 2, 3],
            texture_id: egui::TextureId::default(),
        };
        painter.add(egui::Shape::mesh(mesh));
    }

    // Line
    for window in points.windows(2) {
        painter.line_segment([window[0], window[1]], Stroke::new(1.5, color));
    }
}

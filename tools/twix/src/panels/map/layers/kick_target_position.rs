use std::sync::Arc;

use color_eyre::Result;
use eframe::epaint::{Color32, Stroke};

use coordinate_systems::Ground;
use linear_algebra::{Point2, point};
use types::field_dimensions::FieldDimensions;

use crate::{
    panels::map::layer::Layer, robot::Robot, twix_painter::TwixPainter, value_buffer::BufferHandle,
};

pub struct KickTargetPosition {
    target_position: BufferHandle<Option<Point2<Ground>>>,
}

impl Layer<Ground> for KickTargetPosition {
    const NAME: &'static str = "Kick Target Position";

    fn new(robot: Arc<Robot>) -> Self {
        let target_position =
            robot.subscribe_value("Motion.additional_outputs.kick.target_position");
        Self { target_position }
    }

    fn paint(
        &self,
        painter: &TwixPainter<Ground>,
        _field_dimensions: &FieldDimensions,
    ) -> Result<()> {
        if let Some(Some(target_position)) = self.target_position.get_last_value()? {
            let color = Color32::LIGHT_GREEN;
            let stroke = Stroke {
                width: 0.02,
                color: Color32::BLACK,
            };
            let radius = 0.1;
            painter.circle(target_position, radius, color, stroke);
        } else {
            let color = Color32::RED;
            let stroke = Stroke {
                width: 0.02,
                color: Color32::BLACK,
            };
            let radius = 0.1;
            let point: Point2<Ground> = point![0.0, 0.0];
            painter.circle(point, radius, color, stroke);
        }

        Ok(())
    }
}

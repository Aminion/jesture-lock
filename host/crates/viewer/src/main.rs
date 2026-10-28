//! Viewer for recorded sessions. Converts raw LSB to physical units (g, deg/s) for display only.

fn main() -> eframe::Result {
    eframe::run_native(
        "Gesture IMU viewer",
        eframe::NativeOptions::default(),
        Box::new(|_cc| Ok(Box::new(Viewer::default()))),
    )
}

#[derive(Debug, Default)]
struct Viewer {}

impl eframe::App for Viewer {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ui, |_ui| {});
    }
}

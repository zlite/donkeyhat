import sys
import csv
import os
from PySide2.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QListWidget, QListWidgetItem, QDoubleSpinBox, QLabel
from PySide2.QtWebEngineWidgets import QWebEngineView, QWebEnginePage, QWebEngineProfile
from PySide2.QtCore import QUrl, QObject, Slot, Qt
from PySide2.QtWebChannel import QWebChannel

class CustomWebEnginePage(QWebEnginePage):
    def javaScriptConsoleMessage(self, level, message, lineNumber, sourceID):
        print(f"JS: {message}")

class Bridge(QObject):
    @Slot(float, float)
    def addWaypoint(self, lat, lng):
        window.add_waypoint(lat, lng)

    @Slot(int, float, float)
    def updateWaypoint(self, index, lat, lng):
        window.update_waypoint(index, lat, lng)

class WaypointWidget(QWidget):
    def __init__(self, number, lat, lng, speed=10.0, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.number_label = QLabel(f"{number}.")
        self.coords_label = QLabel(f"Lat: {lat:.6f}, Lng: {lng:.6f}")
        self.speed_spinbox = QDoubleSpinBox()
        self.speed_spinbox.setRange(0, 100)
        self.speed_spinbox.setValue(speed)
        self.speed_spinbox.setSuffix(" m/s")

        layout.addWidget(self.number_label)
        layout.addWidget(self.coords_label, 1)
        layout.addWidget(self.speed_spinbox)

    def update_number(self, number):
        self.number_label.setText(f"{number}.")

    def update_coords(self, lat, lng):
        self.coords_label.setText(f"Lat: {lat:.6f}, Lng: {lng:.6f}")

    def get_data(self):
        return {
            "lat": float(self.coords_label.text().split(", ")[0].split(": ")[1]),
            "lng": float(self.coords_label.text().split(", ")[1].split(": ")[1]),
            "speed": self.speed_spinbox.value()
        }

class WaypointCreator(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Donkeycar Waypoint Creator")
        self.setGeometry(100, 100, 1200, 600)

        main_layout = QHBoxLayout()

        self.map_widget = QWebEngineView()
        main_layout.addWidget(self.map_widget, 7)

        sidebar_layout = QVBoxLayout()
        
        self.waypoint_list = QListWidget()
        self.waypoint_list.setDragDropMode(QListWidget.InternalMove)
        self.waypoint_list.model().rowsMoved.connect(self.on_waypoints_reordered)
        sidebar_layout.addWidget(self.waypoint_list)

        delete_button = QPushButton("Delete Selected Waypoint")
        delete_button.clicked.connect(self.delete_selected_waypoint)
        sidebar_layout.addWidget(delete_button)

        save_button = QPushButton("Save Waypoints")
        save_button.clicked.connect(self.save_waypoints)
        sidebar_layout.addWidget(save_button)

        main_layout.addLayout(sidebar_layout, 3)

        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        self.init_map()

    def init_map(self):
        self.channel = QWebChannel()
        self.bridge = Bridge()
        self.channel.registerObject('bridge', self.bridge)

        page = CustomWebEnginePage(self.map_widget)
        page.profile().setHttpUserAgent("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/58.0.3029.110 Safari/537.3")
        page.profile().setHttpAcceptLanguage("en-US,en;q=0.9")
        
        # Enable persistent cookies
        page.profile().setPersistentCookiesPolicy(QWebEngineProfile.AllowPersistentCookies)
        
        self.map_widget.setPage(page)
        self.map_widget.page().setWebChannel(self.channel)

        html_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "map_template.html")
        self.map_widget.setUrl(QUrl.fromLocalFile(html_path))

        self.map_widget.page().featurePermissionRequested.connect(self.handlePermissionRequest)

    def handlePermissionRequest(self, url, feature):
        if feature in (QWebEnginePage.Geolocation, QWebEnginePage.MediaAudioCapture, QWebEnginePage.MediaVideoCapture, QWebEnginePage.MediaAudioVideoCapture):
            self.map_widget.page().setFeaturePermission(url, feature, QWebEnginePage.PermissionGrantedByUser)
        else:
            self.map_widget.page().setFeaturePermission(url, feature, QWebEnginePage.PermissionDeniedByUser)

    def add_waypoint(self, lat, lng):
        waypoint_widget = WaypointWidget(self.waypoint_list.count() + 1, lat, lng)
        item = QListWidgetItem(self.waypoint_list)
        item.setSizeHint(waypoint_widget.sizeHint())
        self.waypoint_list.addItem(item)
        self.waypoint_list.setItemWidget(item, waypoint_widget)
        print(f"Waypoint added: Lat: {lat:.6f}, Lng: {lng:.6f}")

    def update_waypoint(self, index, lat, lng):
        item = self.waypoint_list.item(index)
        waypoint_widget = self.waypoint_list.itemWidget(item)
        waypoint_widget.update_coords(lat, lng)
        print(f"Waypoint updated: Lat: {lat:.6f}, Lng: {lng:.6f}")

    def on_waypoints_reordered(self):
        for i in range(self.waypoint_list.count()):
            item = self.waypoint_list.item(i)
            waypoint_widget = self.waypoint_list.itemWidget(item)
            waypoint_widget.update_number(i + 1)
        self.update_map_markers()

    def delete_selected_waypoint(self):
        selected_items = self.waypoint_list.selectedItems()
        if selected_items:
            for item in selected_items:
                self.waypoint_list.takeItem(self.waypoint_list.row(item))
            self.on_waypoints_reordered()

    def update_map_markers(self):
        waypoints = [self.waypoint_list.itemWidget(self.waypoint_list.item(i)).get_data() 
                     for i in range(self.waypoint_list.count())]
        self.map_widget.page().runJavaScript(f"updateMarkers({waypoints})")

    def save_waypoints(self):
        waypoints = [self.waypoint_list.itemWidget(self.waypoint_list.item(i)).get_data() 
                     for i in range(self.waypoint_list.count())]
        with open('waypoints.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Latitude', 'Longitude', 'Speed (m/s)'])
            for wp in waypoints:
                writer.writerow([wp['lat'], wp['lng'], wp['speed']])
        print("Waypoints saved to waypoints.csv")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WaypointCreator()
    window.show()
    sys.exit(app.exec_())
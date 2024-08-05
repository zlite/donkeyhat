import sys
import csv
import os
import math
import json
from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QListWidget, QListWidgetItem, QDoubleSpinBox, QLabel, QFileDialog, QCheckBox, QMessageBox
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWebEngineCore import QWebEnginePage, QWebEngineProfile, QWebEngineSettings
from PySide6.QtCore import QUrl, QObject, Slot, Qt
from PySide6.QtWebChannel import QWebChannel
from PySide6.QtCore import QTimer

class CustomWebEnginePage(QWebEnginePage):
    def javaScriptConsoleMessage(self, level, message, lineNumber, sourceID):
        print(f"JS Console ({level}): {message} [line {lineNumber}]")

class Bridge(QObject):
    @Slot(float, float)
    def addWaypoint(self, lat, lng):
        window.add_waypoint(lat, lng, is_relative=window.relative_checkbox.isChecked())

    @Slot(int, float, float)
    def updateWaypoint(self, index, lat, lng):
        window.update_waypoint(index, lat, lng)

class WaypointWidget(QWidget):
    def __init__(self, number, lat, lng, throttle=0.2, parent=None):
        super().__init__(parent)
        self.layout = QHBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.number_label = QLabel(f"{number}.")
        self.coords_label = QLabel()
        self.throttle_spinbox = QDoubleSpinBox()
        self.throttle_spinbox.setRange(0, 1)
        self.throttle_spinbox.setSingleStep(0.1)
        self.throttle_spinbox.setValue(throttle)
        self.throttle_spinbox.setDecimals(2)
        self.throttle_spinbox.setFixedWidth(90)
        self.throttle_spinbox.setAlignment(Qt.AlignRight)

        self.layout.addWidget(self.number_label)
        self.layout.addWidget(self.coords_label, 1)
        self.layout.addWidget(QLabel("Throttle:"))
        self.layout.addWidget(self.throttle_spinbox)

        self.abs_lat = lat
        self.abs_lng = lng
        self.rel_lat = 0
        self.rel_lng = 0
        self.is_relative = False
        self.update_coords(lat, lng)

    def update_number(self, number):
        self.number_label.setText(f"{number}.")

    def update_coords(self, lat, lng, relative=False, home_lat=None, home_lng=None):
        self.is_relative = relative
        if relative and home_lat is not None and home_lng is not None:
            self.rel_lat = lat  # lat is already in meters
            self.rel_lng = lng  # lng is already in meters
            self.abs_lat, self.abs_lng = self.meters_to_lat_lon(lat, lng, home_lat, home_lng)
            self.coords_label.setText(f"ΔLat: {lat:.2f}m, ΔLng: {lng:.2f}m")
        else:
            self.abs_lat = lat
            self.abs_lng = lng
            self.rel_lat = 0
            self.rel_lng = 0
            self.coords_label.setText(f"Lat: {lat:.6f}, Lng: {lng:.6f}")

    @staticmethod
    def meters_to_lat_lon(dlat_m, dlng_m, home_lat, home_lng):
        dlat = dlat_m / 111139
        dlng = dlng_m / (111139 * math.cos(math.radians(home_lat)))
        return home_lat + dlat, home_lng + dlng

    def get_data(self):
        return {
            "abs_lat": self.abs_lat,
            "abs_lng": self.abs_lng,
            "rel_lat": self.rel_lat,
            "rel_lng": self.rel_lng,
            "is_relative": self.is_relative,
            "throttle": self.throttle_spinbox.value()
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

        load_button = QPushButton("Load Waypoints")
        load_button.clicked.connect(self.load_waypoints)
        sidebar_layout.addWidget(load_button)

        set_home_button = QPushButton("Set Home to Map Center")
        set_home_button.clicked.connect(self.set_home_to_map_center)
        sidebar_layout.addWidget(set_home_button)

        self.relative_checkbox = QCheckBox("Use Relative Coordinates")
        self.relative_checkbox.stateChanged.connect(self.toggle_relative_coordinates)
        sidebar_layout.addWidget(self.relative_checkbox)

        main_layout.addLayout(sidebar_layout, 3)

        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        self.init_map()
        self.home_lat = None
        self.home_lng = None
        self.waypoints = []
        self.is_relative = False

    def delay_init(self):
        QTimer.singleShot(1000, self.post_map_init)

    def post_map_init(self):
        # Any initialization that needs to happen after the map is loaded
        print("Map should be loaded now")
        # You can add any necessary initialization here

    def init_map(self):
        self.channel = QWebChannel()
        self.bridge = Bridge()
        self.channel.registerObject('bridge', self.bridge)

        profile = QWebEngineProfile.defaultProfile()
        settings = profile.settings()
        settings.setAttribute(QWebEngineSettings.LocalContentCanAccessRemoteUrls, True)
        settings.setAttribute(QWebEngineSettings.LocalContentCanAccessFileUrls, True)
        settings.setAttribute(QWebEngineSettings.AllowGeolocationOnInsecureOrigins, True)
        settings.setAttribute(QWebEngineSettings.AllowRunningInsecureContent, True)

        page = CustomWebEnginePage(profile, self.map_widget)
        page.setWebChannel(self.channel)

        page.featurePermissionRequested.connect(self.handlePermissionRequest)

        self.map_widget.setPage(page)

        html_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "map_template.html")
        self.map_widget.setUrl(QUrl.fromLocalFile(html_path))
 
    def meters_to_lat_lon(self, dlat_m, dlng_m, home_lat, home_lng):
        dlat = dlat_m / 111139
        dlng = dlng_m / (111139 * math.cos(math.radians(home_lat)))
        return home_lat + dlat, home_lng + dlng

    def lat_lon_to_meters(self, lat, lng, home_lat, home_lng):
        dlat_m = (lat - home_lat) * 111139
        dlng_m = (lng - home_lng) * 111139 * math.cos(math.radians(home_lat))
        return dlat_m, dlng_m

    def handlePermissionRequest(self, url, feature):
        if feature == QWebEnginePage.Geolocation:
            self.map_widget.page().setFeaturePermission(url, feature, QWebEnginePage.PermissionGrantedByUser)
        else:
            self.map_widget.page().setFeaturePermission(url, feature, QWebEnginePage.PermissionDeniedByUser)

    def add_waypoint(self, lat, lng, throttle=0.2, is_relative=False):
        if self.waypoint_list.count() == 0:
            self.home_lat, self.home_lng = lat, lng
            is_relative = False
        
        waypoint_widget = WaypointWidget(self.waypoint_list.count() + 1, lat, lng, throttle)
        
        if is_relative:
            rel_lat = (lat - self.home_lat) * 111139
            rel_lng = (lng - self.home_lng) * 111139 * math.cos(math.radians(self.home_lat))
            waypoint_widget.update_coords(rel_lat, rel_lng, relative=True, home_lat=self.home_lat, home_lng=self.home_lng)
        else:
            waypoint_widget.update_coords(lat, lng, relative=False)
        
        item = QListWidgetItem(self.waypoint_list)
        item.setSizeHint(waypoint_widget.sizeHint())
        self.waypoint_list.addItem(item)
        self.waypoint_list.setItemWidget(item, waypoint_widget)
        
        self.update_map_markers()

    def update_waypoint(self, index, lat, lng):
        item = self.waypoint_list.item(index)
        waypoint_widget = self.waypoint_list.itemWidget(item)
        
        if index == 0:
            # First waypoint is always absolute
            waypoint_widget.update_coords(lat, lng, relative=False)
            self.home_lat, self.home_lng = lat, lng
        elif self.relative_checkbox.isChecked():
            # Convert the new absolute position to relative
            dlat_m, dlng_m = self.lat_lon_to_meters(lat, lng, self.home_lat, self.home_lng)
            waypoint_widget.update_coords(dlat_m, dlng_m, relative=True, home_lat=self.home_lat, home_lng=self.home_lng)
        else:
            waypoint_widget.update_coords(lat, lng, relative=False)
        
        self.update_waypoint_display()
        self.update_map_markers()
        print(f"Waypoint {index + 1} updated: Lat: {lat:.6f}, Lng: {lng:.6f}")

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
            self.update_map_markers()

    def update_map_markers(self):
        waypoints = [self.waypoint_list.itemWidget(self.waypoint_list.item(i)).get_data() 
                    for i in range(self.waypoint_list.count())]
        map_waypoints = []
        for wp in waypoints:
            map_waypoints.append({
                "lat": wp['abs_lat'],
                "lng": wp['abs_lng'],
                "throttle": float(wp['throttle'])
            })
        self.map_widget.page().runJavaScript(f"updateMarkers({json.dumps(map_waypoints)});")

    def save_waypoints(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_name, _ = QFileDialog.getSaveFileName(self, "Save Waypoints", "donkey_path.csv", "CSV Files (*.csv)", options=options)
        if file_name:
            waypoints = [self.waypoint_list.itemWidget(self.waypoint_list.item(i)).get_data() 
                        for i in range(self.waypoint_list.count())]
            with open(file_name, 'w', newline='') as f:
                writer = csv.writer(f)
                for i, wp in enumerate(waypoints):
                    if i == 0 or not self.relative_checkbox.isChecked():
                        # Absolute coordinates: 10 decimal places
                        lat = f"{wp['abs_lat']:.10f}"
                        lng = f"{wp['abs_lng']:.10f}"
                    else:
                        # Relative coordinates: 2 decimal places
                        lat = f"{wp['rel_lat']:.2f}"
                        lng = f"{wp['rel_lng']:.2f}"
                    writer.writerow([lat, lng, f"{wp['throttle']:.2f}"])
            print(f"Waypoints saved to {file_name}")

    def load_waypoints(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_name, _ = QFileDialog.getOpenFileName(self, "Load Waypoints", "", "CSV Files (*.csv)", options=options)
        if file_name:
            self.waypoint_list.clear()
            self.home_lat = None
            self.home_lng = None
            
            self.waypoints = []
            self.is_relative = False
            with open(file_name, 'r') as f:
                reader = csv.reader(f)
                for i, row in enumerate(reader):
                    if len(row) == 3:
                        lat, lng, throttle = row
                        if i == 0:
                            # First waypoint is always absolute
                            lat, lng = float(lat), float(lng)
                            self.home_lat, self.home_lng = lat, lng
                        elif '.' in lat and len(lat.split('.')[1]) <= 2:
                            # Relative coordinates have 2 or fewer decimal places
                            self.is_relative = True
                            lat, lng = float(lat), float(lng)
                        else:
                            # Absolute coordinates
                            lat, lng = float(lat), float(lng)
                        throttle = float(throttle)
                        self.waypoints.append((lat, lng, throttle))
            
            self.relative_checkbox.setChecked(self.is_relative)
            self.process_waypoints()
    def set_home_to_map_center(self):
        js_code = """
        (function() {
            var map = getOrInitMap();
            if (map) {
                var center = map.getCenter();
                return JSON.stringify({
                    lat: center.lat,
                    lng: center.lng
                });
            } else {
                console.error("Map not initialized");
                return null;
            }
        })();
        """
        self.map_widget.page().runJavaScript(js_code, 0, self.on_map_center_received)
        print("Requested map center")

    def force_map_center_update(self):
        js_code = "storeMapCenter();"
        self.map_widget.page().runJavaScript(js_code)

    def on_map_center_received(self, result):
        print(f"Received result: {result}")
        try:
            if not result:
                raise ValueError("Received empty result from JavaScript")

            center = json.loads(result)

            if 'lat' not in center or 'lng' not in center:
                raise KeyError(f"Expected 'lat' and 'lng' keys, but got: {center.keys()}")

            new_home_lat, new_home_lng = float(center['lat']), float(center['lng'])
            print(f"New home set to: Lat: {new_home_lat:.6f}, Lng: {new_home_lng:.6f}")
            
            self.set_home_and_redraw_waypoints(new_home_lat, new_home_lng)
                
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")
        except KeyError as e:
            print(f"Error accessing data: {e}")
        except ValueError as e:
            print(f"Value error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")
        finally:
            print(f"Raw received data: {result}")

    def on_map_center_received_fallback(self, result):
        print(f"Fallback received result: {result}")
        try:
            center = json.loads(result)
            new_home_lat, new_home_lng = float(center['lat']), float(center['lng'])
            print(f"New home set to: Lat: {new_home_lat:.6f}, Lng: {new_home_lng:.6f}")
            self.set_home_and_redraw_waypoints(new_home_lat, new_home_lng)
        except Exception as e:
            print(f"Error in fallback method: {e}")

    def set_home_and_redraw_waypoints(self, new_home_lat, new_home_lng):
        if self.waypoint_list.count() == 0:
            return

        old_home_lat, old_home_lng = self.home_lat, self.home_lng
        self.home_lat, self.home_lng = new_home_lat, new_home_lng

        # Update the first waypoint (home)
        first_item = self.waypoint_list.item(0)
        first_waypoint = self.waypoint_list.itemWidget(first_item)
        first_waypoint.update_coords(new_home_lat, new_home_lng, relative=False)

        # Update all other waypoints
        for i in range(1, self.waypoint_list.count()):
            item = self.waypoint_list.item(i)
            waypoint_widget = self.waypoint_list.itemWidget(item)
            data = waypoint_widget.get_data()
            
            if self.relative_checkbox.isChecked():
                # Keep the relative offset
                waypoint_widget.update_coords(data['rel_lat'], data['rel_lng'], relative=True, home_lat=self.home_lat, home_lng=self.home_lng)
            else:
                # Adjust absolute position
                dlat = data['abs_lat'] - old_home_lat
                dlng = data['abs_lng'] - old_home_lng
                new_lat = new_home_lat + dlat
                new_lng = new_home_lng + dlng
                waypoint_widget.update_coords(new_lat, new_lng, relative=False)

        self.update_waypoint_display()
        self.update_map_markers()
        print(f"Waypoints redrawn with new home location: Lat: {self.home_lat:.6f}, Lng: {self.home_lng:.6f}")

    def process_waypoints(self):
        if not self.waypoints:
            return

        # Clear existing waypoints
        self.waypoint_list.clear()

        # Process the first waypoint (always absolute)
        first_wp = self.waypoints[0]
        self.add_waypoint(first_wp[0], first_wp[1], first_wp[2], is_relative=False)
        self.home_lat, self.home_lng = first_wp[0], first_wp[1]

        # Process the rest of the waypoints
        for lat, lng, throttle in self.waypoints[1:]:
            if self.is_relative:
                # Convert relative meters to absolute coordinates
                abs_lat = self.home_lat + lat / 111139
                abs_lng = self.home_lng + lng / (111139 * math.cos(math.radians(self.home_lat)))
                self.add_waypoint(abs_lat, abs_lng, throttle, is_relative=True)
            else:
                self.add_waypoint(lat, lng, throttle, is_relative=False)

        self.update_waypoint_display()

    def toggle_relative_coordinates(self):
        is_relative = self.relative_checkbox.isChecked()
        if self.waypoint_list.count() > 0:
            first_wp = self.waypoint_list.itemWidget(self.waypoint_list.item(0)).get_data()
            self.home_lat, self.home_lng = first_wp['abs_lat'], first_wp['abs_lng']

        for i in range(self.waypoint_list.count()):
            item = self.waypoint_list.item(i)
            waypoint_widget = self.waypoint_list.itemWidget(item)
            data = waypoint_widget.get_data()
            
            if i == 0:
                waypoint_widget.update_coords(data['abs_lat'], data['abs_lng'], relative=False)
            else:
                if is_relative:
                    dlat_m, dlng_m = self.lat_lon_to_meters(data['abs_lat'], data['abs_lng'], self.home_lat, self.home_lng)
                    waypoint_widget.update_coords(dlat_m, dlng_m, relative=True, home_lat=self.home_lat, home_lng=self.home_lng)
                else:
                    waypoint_widget.update_coords(data['abs_lat'], data['abs_lng'], relative=False)

        self.update_map_markers()
        print(f"Toggled to {'relative' if is_relative else 'absolute'} coordinates")

    def update_waypoint_display(self):
        relative = self.relative_checkbox.isChecked()
        for i in range(self.waypoint_list.count()):
            item = self.waypoint_list.item(i)
            waypoint_widget = self.waypoint_list.itemWidget(item)
            data = waypoint_widget.get_data()
            if i == 0:
                # First waypoint is always absolute
                waypoint_widget.update_coords(data['abs_lat'], data['abs_lng'], relative=False)
            elif relative:
                waypoint_widget.update_coords(data['rel_lat'], data['rel_lng'], relative=True, home_lat=self.home_lat, home_lng=self.home_lng)
            else:
                waypoint_widget.update_coords(data['abs_lat'], data['abs_lng'], relative=False)
        
        self.update_map_markers()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WaypointCreator()
    window.show()
    sys.exit(app.exec())
    
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "ord_lidar_driver.h"
#include "ord_lidar_sdk.h"

namespace py = pybind11;

// Helper function to bind full_scan_data_st and one_scan_data_st structures.
void bind_data_structures(py::module &m) {
    py::class_<ordlidar::point_data_t>(m, "PointData")
        .def(py::init<>())
        .def_readwrite("distance", &ordlidar::point_data_t::distance)
        .def_readwrite("angle", &ordlidar::point_data_t::angle)
        .def_readwrite("intensity", &ordlidar::point_data_t::intensity);

    py::class_<ordlidar::full_scan_data_st>(m, "FullScanData")
        .def(py::init<>())
        .def_readwrite("data", &ordlidar::full_scan_data_st::data)
        .def_readwrite("vailtidy_point_num", &ordlidar::full_scan_data_st::vailtidy_point_num)
        .def_readwrite("speed", &ordlidar::full_scan_data_st::speed);

    py::class_<ordlidar::one_scan_data_st>(m, "OneScanData")
        .def(py::init<>())
        .def_readwrite("data", &ordlidar::one_scan_data_st::data)
        .def_readwrite("vailtidy_point_num", &ordlidar::one_scan_data_st::vailtidy_point_num)
        .def_readwrite("speed", &ordlidar::one_scan_data_st::speed);
}

PYBIND11_MODULE(ordlidar, m) {
    m.doc() = "Pybind11 bindings for the ORDLidar C++ SDK";

    bind_data_structures(m);

    py::class_<ordlidar::OrdlidarDriver>(m, "OrdlidarDriver")
        .def(py::init<uint8_t, int>())
        .def("SetSerialPort", &ordlidar::OrdlidarDriver::SetSerialPort)
        .def("Connect", &ordlidar::OrdlidarDriver::Connect)
        .def("Disconnect", &ordlidar::OrdlidarDriver::Disconnect)
        .def("isConnected", &ordlidar::OrdlidarDriver::isConnected)
        .def("GetTimestamp", &ordlidar::OrdlidarDriver::GetTimestamp)
        .def("GetRotationSpeed", &ordlidar::OrdlidarDriver::GetRotationSpeed)
        .def("SetRotationSpeed", &ordlidar::OrdlidarDriver::SetRotationSpeed)
        .def("Activate", &ordlidar::OrdlidarDriver::Activate)
        .def("Deactive", &ordlidar::OrdlidarDriver::Deactive)
        .def("GetFirmwareVersion", [](const ordlidar::OrdlidarDriver &self) {
            std::string firmware_version;
            self.GetFirmwareVersion(firmware_version);
            return firmware_version;
        })
        .def("GrabFullScanBlocking", [](ordlidar::OrdlidarDriver &self, int timeout_ms) {
            ordlidar::full_scan_data_st scan_data;
            if (self.GrabFullScanBlocking(scan_data, timeout_ms)) {
                return py::cast(scan_data);
            }
            throw py::value_error("Failed to grab full scan data.");
        })
        .def("GrabOneScanBlocking", [](ordlidar::OrdlidarDriver &self, int timeout_ms) {
            ordlidar::one_scan_data_st scan_data;
            if (self.GrabOneScanBlocking(scan_data, timeout_ms)) {
                return py::cast(scan_data);
            }
            throw py::value_error("Failed to grab one scan data.");
        })
        .def("GrabFullScan", [](ordlidar::OrdlidarDriver &self) {
            ordlidar::full_scan_data_st scan_data;
            if (self.GrabFullScan(scan_data)) {
                return py::cast(scan_data);
            }
            throw py::value_error("Failed to grab full scan data.");
        })
        .def("GrabOneScan", [](ordlidar::OrdlidarDriver &self) {
            ordlidar::one_scan_data_st scan_data;
            if (self.GrabOneScan(scan_data)) {
                return py::cast(scan_data);
            }
            throw py::value_error("Failed to grab one scan data.");
        });
    
    py::class_<ordlidar::ORDLidar>(m, "ORDLidar")
        .def(py::init<>())
        .def_readwrite("lidar", &ordlidar::ORDLidar::lidar);

    m.def("oradar_lidar_create", &ordlidar::oradar_lidar_create, py::return_value_policy::take_ownership);
    m.def("oradar_lidar_destroy", &ordlidar::oradar_lidar_destroy);
    m.def("oradar_set_serial_port", &ordlidar::oradar_set_serial_port);
    m.def("oradar_connect", &ordlidar::oradar_connect);
    m.def("oradar_disconnect", &ordlidar::oradar_disconnect);
}


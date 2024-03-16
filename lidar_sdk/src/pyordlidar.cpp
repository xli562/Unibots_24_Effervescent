#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // For automatic conversion between C++ and Python STL containers if needed

#include "ord_lidar_sdk.h" // Your original SDK header
#include "ord_lidar_driver.h" // Your original driver header

namespace py = pybind11;

PYBIND11_MODULE(ordlidar, m) {
    m.doc() = "Pybind11 plugin for ORDLidar";

    // Binding for the ORDLidar struct
    py::class_<ORDLidar>(m, "ORDLidar")
        .def(py::init<>())
        .def_readwrite("lidar", &ORDLidar::lidar);
        
    // Binding for the OrdlidarDriver class
    py::class_<OrdlidarDriver>(m, "OrdlidarDriver")
        .def(py::init<uint8_t, int>())
        .def("SetSerialPort", &OrdlidarDriver::SetSerialPort)
        .def("Connect", &OrdlidarDriver::Connect)
        .def("Disconnect", &OrdlidarDriver::Disconnect)
        .def("GetTimestamp", &OrdlidarDriver::GetTimestamp)
        .def("GetRotationSpeed", &OrdlidarDriver::GetRotationSpeed)
        .def("SetRotationSpeed", &OrdlidarDriver::SetRotationSpeed)
        .def("Activate", &OrdlidarDriver::Activate)
        .def("Deactive", &OrdlidarDriver::Deactive)
        .def("GetFirmwareVersion", &OrdlidarDriver::GetFirmwareVersion)
        .def("GrabFullScanBlocking", &ordlidar::OrdlidarDriver::GrabFullScanBlocking)
        .def("GrabOneScanBlocking", &ordlidar::OrdlidarDriver::GrabOneScanBlocking)
        .def("GrabFullScan", &ordlidar::OrdlidarDriver::GrabFullScan)
        .def("GrabOneScan", &ordlidar::OrdlidarDriver::GrabOneScan);
        
        // You will need to provide custom bindings for functions accepting or returning non-primitive types
        // For example, handling std::string parameters or return values.
        ;

    // Binding for utility functions
    m.def("oradar_lidar_create", &oradar_lidar_create, "Function to create a lidar instance");
    m.def("oradar_lidar_destroy", &oradar_lidar_destroy, "Function to destroy a lidar instance");
    // Continue for other utility functions

    // Note: For complex types or if the function requires manual conversion, you'll need to write custom wrappers.
}
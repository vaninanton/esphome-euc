// Сканер BLE для macOS: выводит имя устройства и его CoreBluetooth-UUID.
// CoreBluetooth не отдаёт MAC — только UUID, локальный для этого Mac.
// Чтобы получить MAC, найди здесь UUID нужного имени, затем:
//   log show --last 30m --predicate 'subsystem == "com.apple.bluetooth"' --info \
//     | grep '<UUID>' | grep 'associated with device'
// Запуск: swift tools/ble-name-scan.swift
import Foundation
import CoreBluetooth

final class Scanner: NSObject, CBCentralManagerDelegate {
    var central: CBCentralManager!
    var seen = Set<String>()

    override init() {
        super.init()
        central = CBCentralManager(delegate: self, queue: nil)
    }

    func centralManagerDidUpdateState(_ c: CBCentralManager) {
        print("state=\(c.state.rawValue) (5=poweredOn, 3=unauthorized)")
        if c.state == .poweredOn {
            c.scanForPeripherals(withServices: nil, options: nil)
        }
    }

    func centralManager(_ c: CBCentralManager, didDiscover p: CBPeripheral,
                        advertisementData: [String: Any], rssi RSSI: NSNumber) {
        let name = p.name ?? (advertisementData[CBAdvertisementDataLocalNameKey] as? String) ?? "-"
        if seen.insert(p.identifier.uuidString).inserted {
            print("\(p.identifier.uuidString)  rssi=\(RSSI)  name=\(name)")
        }
    }
}

let scanner = Scanner()
RunLoop.main.run(until: Date(timeIntervalSinceNow: 25))
print("--- done, \(scanner.seen.count) devices")

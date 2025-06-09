import Serde


public struct MiniData: Hashable {
    @Indirect public var timeSinceBoot: UInt64
    @Indirect public var msgNum: UInt32
    @Indirect public var lat: Double
    @Indirect public var lon: Double
    @Indirect public var alt: Double
    @Indirect public var numSats: UInt8
    @Indirect public var gpsFix: UInt8
    @Indirect public var gpsTime: MiniData.UTC
    @Indirect public var baroAlt: Float
    @Indirect public var ismAxelX: Double
    @Indirect public var ismAxelY: Double
    @Indirect public var ismAxelZ: Double
    @Indirect public var ismGyroX: Double
    @Indirect public var ismGyroY: Double
    @Indirect public var ismGyroZ: Double
    @Indirect public var lsmAxelX: Double
    @Indirect public var lsmAxelY: Double
    @Indirect public var lsmAxelZ: Double
    @Indirect public var lsmGyroX: Double
    @Indirect public var lsmGyroY: Double
    @Indirect public var lsmGyroZ: Double
    @Indirect public var adxlAxelX: Float
    @Indirect public var adxlAxelY: Float
    @Indirect public var adxlAxelZ: Float
    @Indirect public var ismAxelX2: Double
    @Indirect public var ismAxelY2: Double
    @Indirect public var ismAxelZ2: Double
    @Indirect public var ismGyroX2: Double
    @Indirect public var ismGyroY2: Double
    @Indirect public var ismGyroZ2: Double

    public init(timeSinceBoot: UInt64, msgNum: UInt32, lat: Double, lon: Double, alt: Double, numSats: UInt8, gpsFix: UInt8, gpsTime: MiniData.UTC, baroAlt: Float, ismAxelX: Double, ismAxelY: Double, ismAxelZ: Double, ismGyroX: Double, ismGyroY: Double, ismGyroZ: Double, lsmAxelX: Double, lsmAxelY: Double, lsmAxelZ: Double, lsmGyroX: Double, lsmGyroY: Double, lsmGyroZ: Double, adxlAxelX: Float, adxlAxelY: Float, adxlAxelZ: Float, ismAxelX2: Double, ismAxelY2: Double, ismAxelZ2: Double, ismGyroX2: Double, ismGyroY2: Double, ismGyroZ2: Double) {
        self.timeSinceBoot = timeSinceBoot
        self.msgNum = msgNum
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.numSats = numSats
        self.gpsFix = gpsFix
        self.gpsTime = gpsTime
        self.baroAlt = baroAlt
        self.ismAxelX = ismAxelX
        self.ismAxelY = ismAxelY
        self.ismAxelZ = ismAxelZ
        self.ismGyroX = ismGyroX
        self.ismGyroY = ismGyroY
        self.ismGyroZ = ismGyroZ
        self.lsmAxelX = lsmAxelX
        self.lsmAxelY = lsmAxelY
        self.lsmAxelZ = lsmAxelZ
        self.lsmGyroX = lsmGyroX
        self.lsmGyroY = lsmGyroY
        self.lsmGyroZ = lsmGyroZ
        self.adxlAxelX = adxlAxelX
        self.adxlAxelY = adxlAxelY
        self.adxlAxelZ = adxlAxelZ
        self.ismAxelX2 = ismAxelX2
        self.ismAxelY2 = ismAxelY2
        self.ismAxelZ2 = ismAxelZ2
        self.ismGyroX2 = ismGyroX2
        self.ismGyroY2 = ismGyroY2
        self.ismGyroZ2 = ismGyroZ2
    }

    public func serialize<S: Serializer>(serializer: S) throws {
        try serializer.increase_container_depth()
        try serializer.serialize_u64(value: self.timeSinceBoot)
        try serializer.serialize_u32(value: self.msgNum)
        try serializer.serialize_f64(value: self.lat)
        try serializer.serialize_f64(value: self.lon)
        try serializer.serialize_f64(value: self.alt)
        try serializer.serialize_u8(value: self.numSats)
        try serializer.serialize_u8(value: self.gpsFix)
        try self.gpsTime.serialize(serializer: serializer)
        try serializer.serialize_f32(value: self.baroAlt)
        try serializer.serialize_f64(value: self.ismAxelX)
        try serializer.serialize_f64(value: self.ismAxelY)
        try serializer.serialize_f64(value: self.ismAxelZ)
        try serializer.serialize_f64(value: self.ismGyroX)
        try serializer.serialize_f64(value: self.ismGyroY)
        try serializer.serialize_f64(value: self.ismGyroZ)
        try serializer.serialize_f64(value: self.lsmAxelX)
        try serializer.serialize_f64(value: self.lsmAxelY)
        try serializer.serialize_f64(value: self.lsmAxelZ)
        try serializer.serialize_f64(value: self.lsmGyroX)
        try serializer.serialize_f64(value: self.lsmGyroY)
        try serializer.serialize_f64(value: self.lsmGyroZ)
        try serializer.serialize_f32(value: self.adxlAxelX)
        try serializer.serialize_f32(value: self.adxlAxelY)
        try serializer.serialize_f32(value: self.adxlAxelZ)
        try serializer.serialize_f64(value: self.ismAxelX2)
        try serializer.serialize_f64(value: self.ismAxelY2)
        try serializer.serialize_f64(value: self.ismAxelZ2)
        try serializer.serialize_f64(value: self.ismGyroX2)
        try serializer.serialize_f64(value: self.ismGyroY2)
        try serializer.serialize_f64(value: self.ismGyroZ2)
        try serializer.decrease_container_depth()
    }

    public func bincodeSerialize() throws -> [UInt8] {
        let serializer = BincodeSerializer.init();
        try self.serialize(serializer: serializer)
        return serializer.get_bytes()
    }

    public static func deserialize<D: Deserializer>(deserializer: D) throws -> MiniData {
        try deserializer.increase_container_depth()
        let timeSinceBoot = try deserializer.deserialize_u64()
        let msgNum = try deserializer.deserialize_u32()
        let lat = try deserializer.deserialize_f64()
        let lon = try deserializer.deserialize_f64()
        let alt = try deserializer.deserialize_f64()
        let numSats = try deserializer.deserialize_u8()
        let gpsFix = try deserializer.deserialize_u8()
        let gpsTime = try MiniData.UTC.deserialize(deserializer: deserializer)
        let baroAlt = try deserializer.deserialize_f32()
        let ismAxelX = try deserializer.deserialize_f64()
        let ismAxelY = try deserializer.deserialize_f64()
        let ismAxelZ = try deserializer.deserialize_f64()
        let ismGyroX = try deserializer.deserialize_f64()
        let ismGyroY = try deserializer.deserialize_f64()
        let ismGyroZ = try deserializer.deserialize_f64()
        let lsmAxelX = try deserializer.deserialize_f64()
        let lsmAxelY = try deserializer.deserialize_f64()
        let lsmAxelZ = try deserializer.deserialize_f64()
        let lsmGyroX = try deserializer.deserialize_f64()
        let lsmGyroY = try deserializer.deserialize_f64()
        let lsmGyroZ = try deserializer.deserialize_f64()
        let adxlAxelX = try deserializer.deserialize_f32()
        let adxlAxelY = try deserializer.deserialize_f32()
        let adxlAxelZ = try deserializer.deserialize_f32()
        let ismAxelX2 = try deserializer.deserialize_f64()
        let ismAxelY2 = try deserializer.deserialize_f64()
        let ismAxelZ2 = try deserializer.deserialize_f64()
        let ismGyroX2 = try deserializer.deserialize_f64()
        let ismGyroY2 = try deserializer.deserialize_f64()
        let ismGyroZ2 = try deserializer.deserialize_f64()
        try deserializer.decrease_container_depth()
        return MiniData.init(timeSinceBoot: timeSinceBoot, msgNum: msgNum, lat: lat, lon: lon, alt: alt, numSats: numSats, gpsFix: gpsFix, gpsTime: gpsTime, baroAlt: baroAlt, ismAxelX: ismAxelX, ismAxelY: ismAxelY, ismAxelZ: ismAxelZ, ismGyroX: ismGyroX, ismGyroY: ismGyroY, ismGyroZ: ismGyroZ, lsmAxelX: lsmAxelX, lsmAxelY: lsmAxelY, lsmAxelZ: lsmAxelZ, lsmGyroX: lsmGyroX, lsmGyroY: lsmGyroY, lsmGyroZ: lsmGyroZ, adxlAxelX: adxlAxelX, adxlAxelY: adxlAxelY, adxlAxelZ: adxlAxelZ, ismAxelX2: ismAxelX2, ismAxelY2: ismAxelY2, ismAxelZ2: ismAxelZ2, ismGyroX2: ismGyroX2, ismGyroY2: ismGyroY2, ismGyroZ2: ismGyroZ2)
    }

    public static func bincodeDeserialize(input: [UInt8]) throws -> MiniData {
        let deserializer = BincodeDeserializer.init(input: input);
        let obj = try deserialize(deserializer: deserializer)
        if deserializer.get_buffer_offset() < input.count {
            throw DeserializationError.invalidInput(issue: "Some input bytes were not read")
        }
        return obj
    }
}

public struct UTC: Hashable {
    @Indirect public var itow: UInt32
    @Indirect public var timeAccuracyEstimateNs: UInt32
    @Indirect public var nanos: Int32
    @Indirect public var year: UInt16
    @Indirect public var month: UInt8
    @Indirect public var day: UInt8
    @Indirect public var hour: UInt8
    @Indirect public var min: UInt8
    @Indirect public var sec: UInt8
    @Indirect public var valid: UInt8

    public init(itow: UInt32, timeAccuracyEstimateNs: UInt32, nanos: Int32, year: UInt16, month: UInt8, day: UInt8, hour: UInt8, min: UInt8, sec: UInt8, valid: UInt8) {
        self.itow = itow
        self.timeAccuracyEstimateNs = timeAccuracyEstimateNs
        self.nanos = nanos
        self.year = year
        self.month = month
        self.day = day
        self.hour = hour
        self.min = min
        self.sec = sec
        self.valid = valid
    }

    public func serialize<S: Serializer>(serializer: S) throws {
        try serializer.increase_container_depth()
        try serializer.serialize_u32(value: self.itow)
        try serializer.serialize_u32(value: self.timeAccuracyEstimateNs)
        try serializer.serialize_i32(value: self.nanos)
        try serializer.serialize_u16(value: self.year)
        try serializer.serialize_u8(value: self.month)
        try serializer.serialize_u8(value: self.day)
        try serializer.serialize_u8(value: self.hour)
        try serializer.serialize_u8(value: self.min)
        try serializer.serialize_u8(value: self.sec)
        try serializer.serialize_u8(value: self.valid)
        try serializer.decrease_container_depth()
    }

    public func bincodeSerialize() throws -> [UInt8] {
        let serializer = BincodeSerializer.init();
        try self.serialize(serializer: serializer)
        return serializer.get_bytes()
    }

    public static func deserialize<D: Deserializer>(deserializer: D) throws -> UTC {
        try deserializer.increase_container_depth()
        let itow = try deserializer.deserialize_u32()
        let timeAccuracyEstimateNs = try deserializer.deserialize_u32()
        let nanos = try deserializer.deserialize_i32()
        let year = try deserializer.deserialize_u16()
        let month = try deserializer.deserialize_u8()
        let day = try deserializer.deserialize_u8()
        let hour = try deserializer.deserialize_u8()
        let min = try deserializer.deserialize_u8()
        let sec = try deserializer.deserialize_u8()
        let valid = try deserializer.deserialize_u8()
        try deserializer.decrease_container_depth()
        return UTC.init(itow: itow, timeAccuracyEstimateNs: timeAccuracyEstimateNs, nanos: nanos, year: year, month: month, day: day, hour: hour, min: min, sec: sec, valid: valid)
    }

    public static func bincodeDeserialize(input: [UInt8]) throws -> UTC {
        let deserializer = BincodeDeserializer.init(input: input);
        let obj = try deserialize(deserializer: deserializer)
        if deserializer.get_buffer_offset() < input.count {
            throw DeserializationError.invalidInput(issue: "Some input bytes were not read")
        }
        return obj
    }
}


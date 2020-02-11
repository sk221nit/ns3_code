/*
 * bsm-header.cc
 *
 *  Created on: May 22, 2017
 *      Author: Saurabh Kumar
 *      E-mail: saurabh@cbnu.ac.kr
 */
#include "bsm-header.h"
#include <ns3/log.h>
#include <ns3/abort.h>
#include <ns3/node.h>
#include <ns3/spectrum-channel.h>
#include <ns3/pointer.h>
#include <ns3/boolean.h>
#include <ns3/mobility-model.h>
#include <ns3/packet.h>
#include <ns3/simulator.h>
#include <ns3/address-utils.h>
#include <ns3/vector.h>
namespace ns3 {

NS_LOG_COMPONENT_DEFINE("BSMHeader");

NS_OBJECT_ENSURE_REGISTERED(BSMHeader);

BSMHeader::BSMHeader() {

}
TypeId BSMHeader::GetTypeId() {
	static TypeId id =
			TypeId("NS3::BSMHeader").SetParent<Header>().SetGroupName("Wave").AddConstructor<
					BSMHeader>();
	return id;
}
TypeId BSMHeader::GetInstanceTypeId() const {
	return GetTypeId();
}
void BSMHeader::Print(std::ostream &os) const {
	std::cout << "BSM PACKET[ Sequence Number " << (int) m_sequence_number
			<< " Node Id " << (int) m_nodeId << " Generation time "
			<< (int) m_time << " Lattitude : " << m_lattitude << " Longitude : "
			<< m_longitude << " Height " << (int) m_elev << " GPS Accuracy "
			<< m_accuracy << " Speed " << (int) m_speed << " Heading "
			<< (int) m_heading << " Steering Angle " << (int)m_steering_angle
			<< " Long Acc. " << (int)m_acceleration_long << " Lati Acc. "
			<< (int)m_acceleration_lati << " Ver. Acc. " << (int)m_acceleration_verti
			<< " Yaw rate " << (int)m_yaw_rate << " Brake " << (int)m_brake << " Width "
			<< (int)m_width << " Length " << (int)m_length << " Lane Number " << (int)m_lane
			<< " ]" << std::endl;
}
uint32_t BSMHeader::GetSerializedSize() const {
	return 1 + 4 + 2 + 4 + 4 + 2 + 4 + 2 + 2 + 1 + 2 + 2 + 1 + 2 + 2 + 3 + 3 + 1; //Last is lane information
}
void BSMHeader::Serialize(Buffer::Iterator i) const {
	/*
	 * Blob part
	 * Message count ~ 1 Byte
	 * Temporary Id ~ 4 Bytes
	 * DSecond(SecMark) ~ 2 Bytes
	 * Latitude ~ 4 Bytes
	 * Longitude ~ 4 Bytes
	 * elevation ~ 2 Bytes
	 * Positional Accuracy(GPS) ~ 4 Bytes
	 * Transmission Speed ~ 2 Bytes
	 * Heading ~ 2 Bytes
	 * Steering wheel angle ~ 1 Byte
	 * Longitudinal Acceleration ~ 2 Bytes
	 * Latteral Acceleration ~ 2 Bytes
	 * Vertical Acceleration ~ 1 Byte
	 * Acceleration Yaw Rate ~ 2 Bytes
	 * Brake System Status(Brake) ~ 2 Bytes
	 * Width ~ 3 Bytes
	 * Length ~ 3 Bytes
	 *
	 * Additional 1 Byte Lane information
	 *
	 */
	i.WriteU8(m_sequence_number);
	i.WriteU32(m_nodeId);
	i.WriteU16(m_time);
	i.WriteU32(uint32_t(m_lattitude * 1000));
	i.WriteU32(uint32_t(m_longitude * 1000));
	i.WriteU16(m_elev);
	i.WriteU32(uint32_t(m_accuracy * 1000));
	i.WriteU16(m_speed);
	i.WriteU16(m_heading);
	i.WriteU8(m_steering_angle);
	i.WriteU16(m_acceleration_lati);
	i.WriteU16(m_acceleration_long);
	i.WriteU8(m_acceleration_verti);
	i.WriteU16(m_yaw_rate);
	i.WriteU16(m_brake);
	i.WriteU16(m_width);
	i.WriteU8(m_width_dummy);
	i.WriteU16(m_length);
	i.WriteU8(m_length_dummy);
	i.WriteU8(m_lane);

}
uint32_t BSMHeader::Deserialize(Buffer::Iterator it) {
	m_sequence_number = it.ReadU8();
	m_nodeId = it.ReadU32();
	m_time = it.ReadU16();
	m_lattitude = it.ReadU32();
	m_lattitude = m_lattitude/1000;
	m_longitude = it.ReadU32();
	m_longitude = m_longitude/ 1000;
	m_elev = it.ReadU16();
	m_accuracy = it.ReadU32();
	m_accuracy = m_accuracy/ 1000;
	m_speed = it.ReadU16();
	m_heading = it.ReadU16();
	m_steering_angle = it.ReadU8();
	m_acceleration_lati = it.ReadU16();
	m_acceleration_long = it.ReadU16();
	m_acceleration_verti = it.ReadU8();
	m_yaw_rate = it.ReadU16();
	m_brake = it.ReadU16();
	m_width = it.ReadU16();
	m_width_dummy = it.ReadU8();
	m_length = it.ReadU16();
	m_length_dummy = it.ReadU8();
	m_lane = it.ReadU8();
	return GetSerializedSize();
}

void BSMHeader::SetSequenceNumber(uint8_t seq) {
	m_sequence_number = seq;
}
uint8_t BSMHeader::GetSequenceNumber(void) const {
	return m_sequence_number;
}
void BSMHeader::SetNodeId(uint32_t id) {
	m_nodeId = id;
}
uint32_t BSMHeader::GetNodeId(void) const{
	return m_nodeId;
}
void BSMHeader::SetTime(uint16_t milliisec){
	m_time = milliisec;
}
uint16_t BSMHeader::GetTime(void) const{
	return m_time;
}
void BSMHeader::SetLattitude(double x){
	m_lattitude = x;
}
double BSMHeader::GetLattitude(void) const{
	return m_lattitude;
}
void BSMHeader::SetLongitude(double y){
	m_longitude = y;
}
double BSMHeader::GetLongitude(void) const{
	return m_longitude;
}
void BSMHeader::SetElevation(uint16_t elev){
	m_elev = elev;
}
uint16_t BSMHeader::GetElevation(void) const{
	return m_elev;
}
void BSMHeader::SetGPSAccuracy(double accuracy){
	m_accuracy = accuracy;
}
double BSMHeader::GetGPSAccuracy(void) const{
	return m_accuracy;
}
void BSMHeader::SetSpeed(uint16_t speed){
	m_speed = speed;
}
uint16_t BSMHeader::GetSpeed(void) const{
	return m_speed;
}

void BSMHeader::SetHeading(uint16_t heading){
	m_heading = heading;
}
uint16_t BSMHeader::GetHeading(void) const{
	return m_heading;
}

void BSMHeader::setSteeringAngle(uint8_t angle){
	m_steering_angle = angle;
}
uint8_t BSMHeader::GetSteeringAngle(void) const{
	return m_steering_angle;
}

void BSMHeader::SetLongitudeAcceleration(uint16_t acc_x){
	m_acceleration_lati = acc_x;
}
uint16_t BSMHeader::GetLongitudeAcceleration(void) const{
	return m_acceleration_lati;
}

void BSMHeader::SetLattitudeAcceleration(uint16_t acc_y){
	m_acceleration_long = acc_y;
}
uint16_t BSMHeader::GetLattitudeAcceleration(void) const{
	return m_acceleration_long;
}

void BSMHeader::SetVerticleAcceleration(uint8_t acc_z){
	m_acceleration_verti = acc_z;
}
uint8_t BSMHeader::GetVerticleAcceleration(void) const{
	return m_acceleration_verti;
}

void BSMHeader::SetYawRate(uint16_t yaw){
	m_yaw_rate = yaw;
}
uint16_t BSMHeader::GetYawRate(void) const{
	return m_yaw_rate;
}

void BSMHeader::SetBrake(uint16_t brake){
	m_brake = brake;
}
uint16_t BSMHeader::GetBrake(void) const{
	return m_brake;
}

void BSMHeader::SetWidth(uint16_t width){
	m_width = width;
}
uint16_t BSMHeader::GetWidth(void) const{
	return m_width;
}
void BSMHeader::Setlength(uint16_t length){
	m_length = length;
}
uint16_t BSMHeader::GetLength(void) const{
	return m_length;
}

void BSMHeader::SetLane(uint8_t lane){
	m_lane = lane;
}
uint8_t BSMHeader::GetLane(void) const{
	return m_lane;
}

}  // namespace ns3


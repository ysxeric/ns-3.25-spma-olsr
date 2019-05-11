/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */

#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/header.h"
#include "ns3/simulator.h"
#include "seq-ts-header.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SeqTsHeader");

NS_OBJECT_ENSURE_REGISTERED (SeqTsHeader);

SeqTsHeader::SeqTsHeader ()
  : m_seq (0),
    m_ts (Simulator::Now ().GetTimeStep ()),
	m_priority(0)
{
  NS_LOG_FUNCTION (this);
}

void
SeqTsHeader::SetSeq (uint32_t seq)
{
  NS_LOG_FUNCTION (this << seq);
  m_seq = seq;
}

void
SeqTsHeader::SetPriority (uint32_t priority)
{
  NS_LOG_FUNCTION (this << priority);
  m_priority = priority;
}
void
SeqTsHeader::SetSource (uint32_t source)
{
  NS_LOG_FUNCTION (this << source);
  m_source = source;
}

void
SeqTsHeader::SetTime (double time)
{
  NS_LOG_FUNCTION (this << time);
  m_time = time;
}
uint32_t
SeqTsHeader::GetSeq (void) const
{
  NS_LOG_FUNCTION (this);
  return m_seq;
}

uint32_t
SeqTsHeader::GetPriority (void) const
{
  NS_LOG_FUNCTION (this);
  return m_priority;
}
uint32_t
SeqTsHeader::GetSource (void) const
{
  NS_LOG_FUNCTION (this);
  return m_source;
}

double
SeqTsHeader::GetTime (void) const
{
  NS_LOG_FUNCTION (this);
  return m_time;
}

Time
SeqTsHeader::GetTs (void) const
{
  NS_LOG_FUNCTION (this);
  return TimeStep (m_ts);
}

TypeId
SeqTsHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SeqTsHeader")
    .SetParent<Header> ()
    .SetGroupName("Applications")
    .AddConstructor<SeqTsHeader> ()
  ;
  return tid;
}
TypeId
SeqTsHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
void
SeqTsHeader::Print (std::ostream &os) const
{
  NS_LOG_FUNCTION (this << &os);
  os << "(seq=" << m_seq << " time=" << TimeStep (m_ts).GetSeconds () << ")";
}
uint32_t
SeqTsHeader::GetSerializedSize (void) const
{
  NS_LOG_FUNCTION (this);
  return 4+8+4+4;
}

void
SeqTsHeader::Serialize (Buffer::Iterator start) const
{
  NS_LOG_FUNCTION (this << &start);
  Buffer::Iterator i = start;
  i.WriteHtonU32 (m_seq);
  i.WriteHtonU64 (m_ts);
  i.WriteHtonU32 (m_priority);
  i.WriteHtonU32 (m_source);
}
uint32_t
SeqTsHeader::Deserialize (Buffer::Iterator start)
{
  NS_LOG_FUNCTION (this << &start);
  Buffer::Iterator i = start;
  m_seq = i.ReadNtohU32 ();
  m_ts = i.ReadNtohU64 ();
  m_priority = i.ReadNtohU32 ();
  m_source = i.ReadNtohU32 ();
  return GetSerializedSize ();
}

} // namespace ns3

/***********************************************************************************************************************
*                                                                                                                      *
* MCXScope driver for libscopehal / ngscopeclient                                                                      *
*                                                                                                                      *
* Talks to an NXP MCXA156-based headless oscilloscope using the protocol in PROTOCOL.md (v1).                          *
* Redistributable under the same BSD 3-clause terms as libscopehal.                                                    *
*                                                                                                                      *
***********************************************************************************************************************/

/*
 * Notes
 * =====
 * - All voltages on the wire are probe-tip volts; the device owns front-end scaling and calibration,
 *   so attenuation is fixed at 1x and bandwidth limit is "none" from the host's point of view.
 * - Protocol offset is "voltage at screen centre"; scopehal's offset is the value *added* to the signal,
 *   so scopehal_offset = -protocol_offset.
 * - RUN mode re-arms on the device after each readout, so AcquireData() never has to re-arm.
 * - The trigger is placed in AUTO mode on connect so a waveform shows up even with no signal;
 *   change kDefaultTriggerMode below to "NORM" if you prefer classic normal-mode behaviour.
 */

#include "scopehal.h"
#include "MCXScopeOscilloscope.h"
#include "EdgeTrigger.h"

using namespace std;

static const char* kDefaultTriggerMode = "AUTO";

//Fixed part of the waveform record header (PROTOCOL.md section 8.2)
static const size_t kRecordFixedHeader = 40;
static const size_t kRecordChannelEntry = 8;

static uint16_t rd16(const uint8_t* p) { return p[0] | (p[1] << 8); }
static uint32_t rd32(const uint8_t* p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint64_t rd64(const uint8_t* p) { return rd32(p) | ((uint64_t)rd32(p + 4) << 32); }
static float rdf32(const uint8_t* p) { uint32_t u = rd32(p); float f; memcpy(&f, &u, 4); return f; }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

MCXScopeOscilloscope::MCXScopeOscilloscope(SCPITransport* transport)
	: SCPIDevice(transport)
	, SCPIInstrument(transport)
	, m_analogChannelCount(0)
	, m_sampleRate(1)
	, m_sampleDepth(1)
	, m_triggerPosition(0.5)
	, m_triggerOffset(0)
	, m_haveSequence(false)
	, m_lastSequence(0)
{
	m_triggerArmed = false;
	m_triggerOneShot = false;

	if(m_model.find("MCXScope") != 0)
		LogWarning("MCXScope: unexpected model \"%s\" in *IDN? reply, continuing anyway\n", m_model.c_str());

	Command("*CLS");

	string proto = Query(":SYST:PROT?");
	if(proto != "1")
		LogWarning("MCXScope: device reports protocol version \"%s\", driver implements 1\n", proto.c_str());

	//Capabilities
	m_analogChannelCount = stoi(Query(":SYST:CHAN?"));
	if(m_analogChannelCount < 1 || m_analogChannelCount > 16)
	{
		LogError("MCXScope: bad channel count %u\n", m_analogChannelCount);
		m_analogChannelCount = 0;
	}

	for(auto& s : SplitList(Query(":ACQ:RATE:LIST?")))
		m_rates.push_back(stoull(s));
	for(auto& s : SplitList(Query(":ACQ:DEPT:LIST?")))
		m_depths.push_back(stoull(s));

	static const char* colors[] = { "#ffff00", "#00ffff", "#ff00ff", "#00ff00",
		"#ff8000", "#8080ff", "#ff4040", "#ffffff" };

	m_couplings.resize(m_analogChannelCount);
	m_channelEnabled.resize(m_analogChannelCount, false);
	m_channelRange.resize(m_analogChannelCount, 1);
	m_channelOffset.resize(m_analogChannelCount, 0);
	m_channelCoupling.resize(m_analogChannelCount, OscilloscopeChannel::COUPLE_DC_1M);

	for(size_t i=0; i<m_analogChannelCount; i++)
	{
		string hwname = "CHAN" + to_string(i+1);

		auto chan = new OscilloscopeChannel(
			this,
			hwname,
			colors[i % (sizeof(colors)/sizeof(colors[0]))],
			Unit(Unit::UNIT_FS),
			Unit(Unit::UNIT_VOLTS),
			Stream::STREAM_TYPE_ANALOG,
			i);
		m_channels.push_back(chan);
		chan->SetDefaultDisplayName();

		for(auto& c : SplitList(Query(":" + hwname + ":COUP:LIST?")))
		{
			if(c == "DC")
				m_couplings[i].push_back(OscilloscopeChannel::COUPLE_DC_1M);
			else if(c == "AC")
				m_couplings[i].push_back(OscilloscopeChannel::COUPLE_AC_1M);
		}

		PullChannel(i);
	}

	Command(string(":TRIG:MODE ") + kDefaultTriggerMode);
	PullTimebase();
	PullTrigger();
	CheckErrors("init");
}

MCXScopeOscilloscope::~MCXScopeOscilloscope()
{
}

string MCXScopeOscilloscope::GetDriverNameInternal()
{
	return "mcxscope";
}

unsigned int MCXScopeOscilloscope::GetInstrumentTypes() const
{
	return Instrument::INST_OSCILLOSCOPE;
}

uint32_t MCXScopeOscilloscope::GetInstrumentTypesForChannel(size_t /*i*/) const
{
	return Instrument::INST_OSCILLOSCOPE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helpers

string MCXScopeOscilloscope::Query(const string& cmd)
{
	return Trim(m_transport->SendCommandQueuedWithReply(cmd, false));
}

void MCXScopeOscilloscope::Command(const string& cmd)
{
	m_transport->SendCommandQueued(cmd);
}

vector<string> MCXScopeOscilloscope::SplitList(const string& s)
{
	vector<string> ret;
	for(auto& part : explode(s, ','))
	{
		auto t = Trim(part);
		if(!t.empty())
			ret.push_back(t);
	}
	return ret;
}

/**
	@brief Drains the device error queue into the log
 */
void MCXScopeOscilloscope::CheckErrors(const char* context)
{
	for(int i=0; i<10; i++)
	{
		string e = Query(":SYST:ERR?");
		if(e.empty() || e[0] == '0')
			return;
		LogWarning("MCXScope (%s): device error %s\n", context, e.c_str());
	}
}

void MCXScopeOscilloscope::PullChannel(size_t i)
{
	string hw = ":CHAN" + to_string(i+1);

	bool en = Query(hw + ":ENAB?") == "1";
	float range = stof(Query(hw + ":RANG?"));
	float centre = stof(Query(hw + ":OFFS?"));
	string coup = Query(hw + ":COUP?");

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelEnabled[i] = en;
	m_channelRange[i] = range;
	m_channelOffset[i] = -centre;
	m_channelCoupling[i] = (coup == "AC") ? OscilloscopeChannel::COUPLE_AC_1M : OscilloscopeChannel::COUPLE_DC_1M;
}

/**
	@brief Re-reads rate, depth and trigger position (depth may change when channels are enabled)
 */
void MCXScopeOscilloscope::PullTimebase()
{
	uint64_t rate = stoull(Query(":ACQ:RATE?"));
	uint64_t depth = stoull(Query(":ACQ:DEPT?"));
	double pos = stod(Query(":TRIG:POS?"));

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_sampleRate = rate ? rate : 1;
	m_sampleDepth = depth;
	m_triggerPosition = pos;
	m_triggerOffset = llround(pos * depth * (FS_PER_SECOND / m_sampleRate));
}

void MCXScopeOscilloscope::FlushConfigCache()
{
	for(size_t i=0; i<m_analogChannelCount; i++)
		PullChannel(i);
	PullTimebase();
	PullTrigger();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Channel configuration

bool MCXScopeOscilloscope::IsChannelEnabled(size_t i)
{
	if(i >= m_analogChannelCount)
		return false;
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	return m_channelEnabled[i];
}

void MCXScopeOscilloscope::EnableChannel(size_t i)
{
	if(i >= m_analogChannelCount)
		return;
	Command(":CHAN" + to_string(i+1) + ":ENAB 1");
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_channelEnabled[i] = true;
	}
	PullTimebase();	//effective depth may have shrunk
}

void MCXScopeOscilloscope::DisableChannel(size_t i)
{
	if(i >= m_analogChannelCount)
		return;
	Command(":CHAN" + to_string(i+1) + ":ENAB 0");
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_channelEnabled[i] = false;
	}
	PullTimebase();
}

vector<OscilloscopeChannel::CouplingType> MCXScopeOscilloscope::GetAvailableCouplings(size_t i)
{
	if(i >= m_analogChannelCount)
		return {};
	return m_couplings[i];
}

OscilloscopeChannel::CouplingType MCXScopeOscilloscope::GetChannelCoupling(size_t i)
{
	if(i >= m_analogChannelCount)
		return OscilloscopeChannel::COUPLE_SYNTHETIC;
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	return m_channelCoupling[i];
}

void MCXScopeOscilloscope::SetChannelCoupling(size_t i, OscilloscopeChannel::CouplingType type)
{
	if(i >= m_analogChannelCount)
		return;

	const char* name;
	switch(type)
	{
		case OscilloscopeChannel::COUPLE_DC_1M:
			name = "DC";
			break;
		case OscilloscopeChannel::COUPLE_AC_1M:
			name = "AC";
			break;
		default:
			LogWarning("MCXScope: unsupported coupling %d\n", type);
			return;
	}
	Command(":CHAN" + to_string(i+1) + ":COUP " + name);
	PullChannel(i);
}

double MCXScopeOscilloscope::GetChannelAttenuation(size_t /*i*/)
{
	//Device reports probe-tip volts; front-end scaling is handled in firmware
	return 1;
}

void MCXScopeOscilloscope::SetChannelAttenuation(size_t /*i*/, double /*atten*/)
{
}

unsigned int MCXScopeOscilloscope::GetChannelBandwidthLimit(size_t /*i*/)
{
	return 0;
}

void MCXScopeOscilloscope::SetChannelBandwidthLimit(size_t /*i*/, unsigned int /*limit_mhz*/)
{
}

float MCXScopeOscilloscope::GetChannelVoltageRange(size_t i, size_t /*stream*/)
{
	if(i >= m_analogChannelCount)
		return 1;
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	return m_channelRange[i];
}

void MCXScopeOscilloscope::SetChannelVoltageRange(size_t i, size_t /*stream*/, float range)
{
	if(i >= m_analogChannelCount)
		return;
	string hw = ":CHAN" + to_string(i+1);
	Command(hw + ":RANG " + to_string_sci(range));

	//Device coerces to a supported range; read back what we actually got
	float actual = stof(Query(hw + ":RANG?"));
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelRange[i] = actual;
}

OscilloscopeChannel* MCXScopeOscilloscope::GetExternalTrigger()
{
	return nullptr;
}

float MCXScopeOscilloscope::GetChannelOffset(size_t i, size_t /*stream*/)
{
	if(i >= m_analogChannelCount)
		return 0;
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	return m_channelOffset[i];
}

void MCXScopeOscilloscope::SetChannelOffset(size_t i, size_t /*stream*/, float offset)
{
	if(i >= m_analogChannelCount)
		return;
	string hw = ":CHAN" + to_string(i+1);
	Command(hw + ":OFFS " + to_string_sci(-offset));

	float centre = stof(Query(hw + ":OFFS?"));
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelOffset[i] = -centre;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Triggering / acquisition

Oscilloscope::TriggerMode MCXScopeOscilloscope::PollTrigger()
{
	string stat = Query(":STAT?");

	if(stat == "TRIGGERED")
	{
		m_triggerArmed = false;
		return TRIGGER_MODE_TRIGGERED;
	}
	else if(stat == "ARMED")
		return TRIGGER_MODE_RUN;
	else
	{
		if(stat != "STOP")
			LogWarning("MCXScope: unexpected status \"%s\"\n", stat.c_str());
		return TRIGGER_MODE_STOP;
	}
}

bool MCXScopeOscilloscope::AcquireData()
{
	vector<uint8_t> payload;

	//Read the binary block under the transport lock so nothing else interleaves with it
	{
		lock_guard<recursive_mutex> lock(m_transport->GetMutex());
		m_transport->FlushCommandQueue();
		m_transport->SendCommand(":WAV:DATA?");

		char hdr[2] = {0};
		if(m_transport->ReadRawData(2, (unsigned char*)hdr) != 2 || hdr[0] != '#' || !isdigit(hdr[1]))
		{
			LogError("MCXScope: bad block header\n");
			m_transport->FlushRXBuffer();
			return false;
		}
		size_t ndigits = hdr[1] - '0';
		char digits[10] = {0};
		if(ndigits && (m_transport->ReadRawData(ndigits, (unsigned char*)digits) != ndigits))
		{
			LogError("MCXScope: short block length\n");
			m_transport->FlushRXBuffer();
			return false;
		}
		size_t len = ndigits ? stoull(digits) : 0;

		payload.resize(len);
		if(len && (m_transport->ReadRawData(len, payload.data()) != len))
		{
			LogError("MCXScope: short block payload (wanted %zu bytes)\n", len);
			m_transport->FlushRXBuffer();
			return false;
		}

		unsigned char lf;
		m_transport->ReadRawData(1, &lf);
	}

	if(payload.empty())
	{
		//Device had nothing ready (status changed under us); log the error and carry on
		CheckErrors("acquire");
		return false;
	}

	//Parse the fixed header
	const uint8_t* p = payload.data();
	if(payload.size() < kRecordFixedHeader || memcmp(p, "MCXW", 4) != 0)
	{
		LogError("MCXScope: bad record magic\n");
		return false;
	}
	uint16_t version = rd16(p + 4);
	size_t headerLen = rd16(p + 6);
	uint16_t mask = rd16(p + 8);
	uint8_t flags = p[11];
	size_t depth = rd32(p + 12);
	uint64_t periodFs = rd64(p + 16);
	uint32_t trigIndex = rd32(p + 24);
	float trigFrac = rdf32(p + 28);
	uint32_t sequence = rd32(p + 32);

	if(version != 1)
		LogWarning("MCXScope: record version %u, parsing as v1\n", version);

	vector<size_t> chans;
	for(size_t i=0; i<16; i++)
	{
		if(mask & (1 << i))
			chans.push_back(i);
	}
	if( (headerLen < kRecordFixedHeader + kRecordChannelEntry * chans.size()) ||
		(payload.size() != headerLen + 2 * depth * chans.size()) ||
		(periodFs == 0) )
	{
		LogError("MCXScope: inconsistent record (len=%zu header=%zu depth=%zu nch=%zu)\n",
			payload.size(), headerLen, depth, chans.size());
		return false;
	}

	if(flags & 0x02)
		LogWarning("MCXScope: device reported sample overrun, data may be corrupt\n");
	if(m_haveSequence && (sequence != m_lastSequence + 1))
		LogDebug("MCXScope: sequence jumped %u -> %u\n", m_lastSequence, sequence);
	m_haveSequence = true;
	m_lastSequence = sequence;

	//Trigger crossing happened at (trigIndex - trigFrac) samples. We report the trigger at exactly
	//trigIndex samples and shift the waveform right by trigFrac so the crossing lands on it.
	int64_t trigPhase = 0;
	if(!(flags & 0x01) && trigFrac > 0 && trigFrac < 1)
		trigPhase = llround(trigFrac * periodFs);
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_triggerOffset = (int64_t)trigIndex * (int64_t)periodFs;
	}

	double t = GetTime();
	int64_t tsec = floor(t);
	int64_t tfs = (t - tsec) * FS_PER_SECOND;

	SequenceSet s;
	for(size_t k=0; k<chans.size(); k++)
	{
		size_t ch = chans[k];
		if(ch >= m_analogChannelCount)
		{
			LogWarning("MCXScope: record contains channel %zu which the device didn't advertise\n", ch+1);
			continue;
		}

		float voltsPerCode = rdf32(p + kRecordFixedHeader + kRecordChannelEntry*k);
		float offsetVolts = rdf32(p + kRecordFixedHeader + kRecordChannelEntry*k + 4);
		const uint8_t* samples = p + headerLen + 2*depth*k;

		auto cap = AllocateAnalogWaveform(m_nickname + "." + GetOscilloscopeChannel(ch)->GetHwname());
		cap->m_timescale = periodFs;
		cap->m_triggerPhase = trigPhase;
		cap->m_startTimestamp = tsec;
		cap->m_startFemtoseconds = tfs;
		cap->Resize(depth);
		cap->PrepareForCpuAccess();
		float* out = cap->m_samples.GetCpuPointer();
		for(size_t j=0; j<depth; j++)
		{
			int16_t code = (int16_t)rd16(samples + 2*j);
			out[j] = code * voltsPerCode + offsetVolts;
		}
		cap->MarkSamplesModifiedFromCpu();

		s[GetOscilloscopeChannel(ch)] = cap;
	}

	m_pendingWaveformsMutex.lock();
	m_pendingWaveforms.push_back(s);
	m_pendingWaveformsMutex.unlock();

	//In RUN mode the device re-armed itself after the readout
	m_triggerArmed = !m_triggerOneShot;
	return true;
}

void MCXScopeOscilloscope::Start()
{
	Command(":RUN");
	m_triggerArmed = true;
	m_triggerOneShot = false;
}

void MCXScopeOscilloscope::StartSingleTrigger()
{
	Command(":SINGLE");
	m_triggerArmed = true;
	m_triggerOneShot = true;
}

void MCXScopeOscilloscope::Stop()
{
	Command(":STOP");
	m_triggerArmed = false;
	m_triggerOneShot = true;
}

void MCXScopeOscilloscope::ForceTrigger()
{
	//Device only honours FORCe while armed
	if(!m_triggerArmed)
		StartSingleTrigger();
	Command(":TRIG:FORC");
}

bool MCXScopeOscilloscope::IsTriggerArmed()
{
	return m_triggerArmed;
}

void MCXScopeOscilloscope::PullTrigger()
{
	PullEdgeTrigger();
}

void MCXScopeOscilloscope::PullEdgeTrigger()
{
	//Replace any trigger of the wrong type
	if( (m_trigger != nullptr) && (dynamic_cast<EdgeTrigger*>(m_trigger) == nullptr) )
	{
		delete m_trigger;
		m_trigger = nullptr;
	}
	if(m_trigger == nullptr)
		m_trigger = new EdgeTrigger(this);
	auto et = dynamic_cast<EdgeTrigger*>(m_trigger);

	string src = Query(":TRIG:SOUR?");
	if( (src.find("CHAN") == 0) && (src.length() > 4) )
	{
		size_t n = stoul(src.substr(4));
		if( (n >= 1) && (n <= m_analogChannelCount) )
			et->SetInput(0, StreamDescriptor(GetOscilloscopeChannel(n-1), 0), true);
		else
			LogWarning("MCXScope: trigger source %s out of range\n", src.c_str());
	}
	else
		LogWarning("MCXScope: unknown trigger source \"%s\"\n", src.c_str());

	et->SetLevel(stof(Query(":TRIG:LEV?")));

	string edge = Query(":TRIG:EDGE?");
	if(edge == "RIS")
		et->SetType(EdgeTrigger::EDGE_RISING);
	else if(edge == "FALL")
		et->SetType(EdgeTrigger::EDGE_FALLING);
	else if(edge == "ANY")
		et->SetType(EdgeTrigger::EDGE_ANY);
	else
		LogWarning("MCXScope: unknown trigger edge \"%s\"\n", edge.c_str());
}

void MCXScopeOscilloscope::PushTrigger()
{
	auto et = dynamic_cast<EdgeTrigger*>(m_trigger);
	if(et)
		PushEdgeTrigger(et);
	else
		LogWarning("MCXScope: only edge triggers are supported\n");
}

void MCXScopeOscilloscope::PushEdgeTrigger(EdgeTrigger* trig)
{
	auto in = trig->GetInput(0);
	if(in.m_channel)
		Command(":TRIG:SOUR CHAN" + to_string(in.m_channel->GetIndex() + 1));

	switch(trig->GetType())
	{
		case EdgeTrigger::EDGE_RISING:
			Command(":TRIG:EDGE RIS");
			break;
		case EdgeTrigger::EDGE_FALLING:
			Command(":TRIG:EDGE FALL");
			break;
		case EdgeTrigger::EDGE_ANY:
			Command(":TRIG:EDGE ANY");
			break;
		default:
			LogWarning("MCXScope: unsupported edge type %d, using rising\n", trig->GetType());
			Command(":TRIG:EDGE RIS");
			break;
	}

	//Level last: the device clamps it to the (possibly new) source channel's range
	Command(":TRIG:LEV " + to_string_sci(trig->GetLevel()));
	trig->SetLevel(stof(Query(":TRIG:LEV?")));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Timebase

vector<uint64_t> MCXScopeOscilloscope::GetSampleRatesNonInterleaved()
{
	return m_rates;
}

vector<uint64_t> MCXScopeOscilloscope::GetSampleRatesInterleaved()
{
	return m_rates;
}

set<Oscilloscope::InterleaveConflict> MCXScopeOscilloscope::GetInterleaveConflicts()
{
	return {};
}

vector<uint64_t> MCXScopeOscilloscope::GetSampleDepthsNonInterleaved()
{
	return m_depths;
}

vector<uint64_t> MCXScopeOscilloscope::GetSampleDepthsInterleaved()
{
	return m_depths;
}

uint64_t MCXScopeOscilloscope::GetSampleRate()
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	return m_sampleRate;
}

uint64_t MCXScopeOscilloscope::GetSampleDepth()
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	return m_sampleDepth;
}

void MCXScopeOscilloscope::SetSampleDepth(uint64_t depth)
{
	Command(":ACQ:DEPT " + to_string(depth));
	PullTimebase();
}

void MCXScopeOscilloscope::SetSampleRate(uint64_t rate)
{
	Command(":ACQ:RATE " + to_string(rate));
	PullTimebase();
}

void MCXScopeOscilloscope::SetTriggerOffset(int64_t offset)
{
	double duration;
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		duration = m_sampleDepth * (FS_PER_SECOND / m_sampleRate);
	}
	double pos = (duration > 0) ? (offset / duration) : 0.5;
	pos = max(0.0, min(1.0, pos));

	char tmp[64];
	snprintf(tmp, sizeof(tmp), ":TRIG:POS %.6f", pos);
	Command(tmp);
	PullTimebase();
}

int64_t MCXScopeOscilloscope::GetTriggerOffset()
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	return m_triggerOffset;
}

bool MCXScopeOscilloscope::IsInterleaving()
{
	return false;
}

bool MCXScopeOscilloscope::SetInterleaving(bool /*combine*/)
{
	return false;
}

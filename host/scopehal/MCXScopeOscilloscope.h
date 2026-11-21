/***********************************************************************************************************************
*                                                                                                                      *
* MCXScope driver for libscopehal / ngscopeclient                                                                      *
*                                                                                                                      *
* Talks to an NXP MCXA156-based headless oscilloscope using the protocol in PROTOCOL.md (v1).                          *
* Redistributable under the same BSD 3-clause terms as libscopehal.                                                    *
*                                                                                                                      *
***********************************************************************************************************************/

#ifndef MCXScopeOscilloscope_h
#define MCXScopeOscilloscope_h

class EdgeTrigger;

/**
	@brief Driver for the MCXScope headless oscilloscope (see PROTOCOL.md)

	Connect with driver "mcxscope" and transport "uart" (path e.g. /dev/ttyACM0) or "lan" (host:5025).
 */
class MCXScopeOscilloscope : public virtual SCPIOscilloscope
{
public:
	MCXScopeOscilloscope(SCPITransport* transport);
	virtual ~MCXScopeOscilloscope();

	//not copyable or assignable
	MCXScopeOscilloscope(const MCXScopeOscilloscope& rhs) =delete;
	MCXScopeOscilloscope& operator=(const MCXScopeOscilloscope& rhs) =delete;

public:
	//Device information
	virtual unsigned int GetInstrumentTypes() const override;
	virtual uint32_t GetInstrumentTypesForChannel(size_t i) const override;

	virtual void FlushConfigCache() override;

	//Channel configuration
	virtual bool IsChannelEnabled(size_t i) override;
	virtual void EnableChannel(size_t i) override;
	virtual void DisableChannel(size_t i) override;
	virtual OscilloscopeChannel::CouplingType GetChannelCoupling(size_t i) override;
	virtual void SetChannelCoupling(size_t i, OscilloscopeChannel::CouplingType type) override;
	virtual std::vector<OscilloscopeChannel::CouplingType> GetAvailableCouplings(size_t i) override;
	virtual double GetChannelAttenuation(size_t i) override;
	virtual void SetChannelAttenuation(size_t i, double atten) override;
	virtual unsigned int GetChannelBandwidthLimit(size_t i) override;
	virtual void SetChannelBandwidthLimit(size_t i, unsigned int limit_mhz) override;
	virtual float GetChannelVoltageRange(size_t i, size_t stream) override;
	virtual void SetChannelVoltageRange(size_t i, size_t stream, float range) override;
	virtual OscilloscopeChannel* GetExternalTrigger() override;
	virtual float GetChannelOffset(size_t i, size_t stream) override;
	virtual void SetChannelOffset(size_t i, size_t stream, float offset) override;

	//Triggering
	virtual Oscilloscope::TriggerMode PollTrigger() override;
	virtual bool AcquireData() override;
	virtual void Start() override;
	virtual void StartSingleTrigger() override;
	virtual void Stop() override;
	virtual void ForceTrigger() override;
	virtual bool IsTriggerArmed() override;
	virtual void PushTrigger() override;
	virtual void PullTrigger() override;

	//Timebase
	virtual std::vector<uint64_t> GetSampleRatesNonInterleaved() override;
	virtual std::vector<uint64_t> GetSampleRatesInterleaved() override;
	virtual std::set<InterleaveConflict> GetInterleaveConflicts() override;
	virtual std::vector<uint64_t> GetSampleDepthsNonInterleaved() override;
	virtual std::vector<uint64_t> GetSampleDepthsInterleaved() override;
	virtual uint64_t GetSampleRate() override;
	virtual uint64_t GetSampleDepth() override;
	virtual void SetSampleDepth(uint64_t depth) override;
	virtual void SetSampleRate(uint64_t rate) override;
	virtual void SetTriggerOffset(int64_t offset) override;
	virtual int64_t GetTriggerOffset() override;
	virtual bool IsInterleaving() override;
	virtual bool SetInterleaving(bool combine) override;

protected:
	std::string Query(const std::string& cmd);
	void Command(const std::string& cmd);
	static std::vector<std::string> SplitList(const std::string& s);

	void PullChannel(size_t i);
	void PullTimebase();
	void PullEdgeTrigger();
	void PushEdgeTrigger(EdgeTrigger* trig);
	void CheckErrors(const char* context);

	unsigned int m_analogChannelCount;

	//Capabilities (fixed for the lifetime of the connection)
	std::vector<uint64_t> m_rates;
	std::vector<uint64_t> m_depths;
	std::vector<std::vector<OscilloscopeChannel::CouplingType>> m_couplings;

	//Config cache (guarded by m_cacheMutex)
	std::vector<bool> m_channelEnabled;
	std::vector<float> m_channelRange;
	std::vector<float> m_channelOffset;		//scopehal convention: offset = -(voltage at screen centre)
	std::vector<OscilloscopeChannel::CouplingType> m_channelCoupling;
	uint64_t m_sampleRate;
	uint64_t m_sampleDepth;
	double m_triggerPosition;				//fraction of record before trigger
	int64_t m_triggerOffset;				//fs from start of record to trigger

	//Acquisition bookkeeping
	bool m_haveSequence;
	uint32_t m_lastSequence;

public:
	static std::string GetDriverNameInternal();

	//This is intentionally not virtual since it's a static method used by enumeration
	//cppcheck-suppress duplInheritedMember
	static std::vector<SCPIInstrumentModel> GetDriverSupportedModels()
	{
		return
		{
			{"MCXScope", {
				{ SCPITransportType::TRANSPORT_UART, "" },
				{ SCPITransportType::TRANSPORT_LAN, "" }
			}}
		};
	}

	OSCILLOSCOPE_INITPROC(MCXScopeOscilloscope)
};

#endif

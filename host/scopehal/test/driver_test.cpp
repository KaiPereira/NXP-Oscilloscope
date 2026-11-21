/*
 * driver_test.cpp: headless end-to-end test of the mcxscope libscopehal driver.
 *
 * Start the emulator first:   python3 host/tools/fake_scope.py --tcp 5025
 * Then run:                   ./driver_test [transport] [path]     (default: lan 127.0.0.1:5025)
 *
 * Build: see build.sh next to this file (links against an ngscopeclient build tree).
 * Expects the emulator's signals: CHAN1 = 1 kHz sine, 1 Vpp, +0.1 V DC; CHAN2 = 250 Hz square, +/-1 V.
 */

#include "scopehal.h"
#include "EdgeTrigger.h"
#include <cmath>

using namespace std;

static int g_pass = 0, g_fail = 0;

static void Check(const char* name, bool ok, const string& detail = "")
{
	if(ok)
		g_pass++;
	else
		g_fail++;
	printf("  %s  %s%s%s\n", ok ? "PASS" : "FAIL", name, detail.empty() ? "" : "  -- ", detail.c_str());
}

static string Fmt(const char* fmt, double a, double b = 0)
{
	char tmp[128];
	snprintf(tmp, sizeof(tmp), fmt, a, b);
	return tmp;
}

/// Waits for a trigger, acquires, and pops the waveform into the channels. Returns false on timeout.
static bool Capture(shared_ptr<Oscilloscope> scope, double timeout = 5)
{
	double start = GetTime();
	while(GetTime() - start < timeout)
	{
		if(scope->PollTrigger() == Oscilloscope::TRIGGER_MODE_TRIGGERED)
		{
			if(!scope->AcquireData())
				return false;
			return scope->PopPendingWaveform();
		}
		this_thread::sleep_for(chrono::milliseconds(5));
	}
	return false;
}

static UniformAnalogWaveform* Wfm(shared_ptr<Oscilloscope> scope, size_t ch)
{
	return dynamic_cast<UniformAnalogWaveform*>(scope->GetOscilloscopeChannel(ch)->GetData(0));
}

int main(int argc, char* argv[])
{
	string transportName = argc > 1 ? argv[1] : "lan";
	string path = argc > 2 ? argv[2] : "127.0.0.1:5025";

	g_log_sinks.emplace(g_log_sinks.begin(), new ColoredSTDLogSink(Severity::NOTICE));

	if(!VulkanInit(true))
	{
		printf("VulkanInit failed\n");
		return 2;
	}
	TransportStaticInit();
	DriverStaticInit();

	printf("== Connect\n");
	auto transport = SCPITransport::CreateTransport(transportName, path);
	if(!transport || !transport->IsConnected())
	{
		printf("could not connect to %s %s\n", transportName.c_str(), path.c_str());
		return 2;
	}
	auto scope = Oscilloscope::CreateOscilloscope("mcxscope", transport);
	Check("driver instantiated", scope != nullptr);
	if(!scope)
		return 1;
	scope->m_nickname = "mcx";
	printf("  connected: %s / %s / %s\n", scope->GetVendor().c_str(), scope->GetName().c_str(), scope->GetSerial().c_str());
	Check("driver name", dynamic_pointer_cast<SCPIInstrument>(scope)->GetDriverName() == "mcxscope");
	Check("model starts with MCXScope", scope->GetName().find("MCXScope") == 0);
	Check("2 analog channels", scope->GetChannelCount() == 2, to_string(scope->GetChannelCount()));
	Check("rate list from device", scope->GetSampleRatesNonInterleaved().size() == 5);
	Check("depth list from device", scope->GetSampleDepthsNonInterleaved().size() == 5);

	printf("== Settings round-trip\n");
	scope->SetSampleRate(1000000);
	Check("sample rate 1 MS/s", scope->GetSampleRate() == 1000000, to_string(scope->GetSampleRate()));
	scope->SetSampleRate(1200000);
	Check("sample rate coerced to 1 MS/s", scope->GetSampleRate() == 1000000, to_string(scope->GetSampleRate()));
	scope->SetSampleDepth(10000);
	Check("depth 10000", scope->GetSampleDepth() == 10000);
	scope->EnableChannel(1);
	Check("CHAN2 enabled", scope->IsChannelEnabled(1));
	scope->SetSampleDepth(20000);
	Check("depth 20000 with 2 channels (fits MAXPoints)", scope->GetSampleDepth() == 20000);
	scope->SetSampleDepth(10000);
	scope->SetChannelVoltageRange(0, 0, 1.5);
	Check("range coerced up to 2 V", fabs(scope->GetChannelVoltageRange(0, 0) - 2) < 1e-6,
		Fmt("%g", scope->GetChannelVoltageRange(0, 0)));
	scope->SetChannelVoltageRange(1, 0, 10);
	scope->SetChannelOffset(0, 0, -0.25);
	Check("offset round-trip (sign convention)", fabs(scope->GetChannelOffset(0, 0) + 0.25) < 1e-6,
		Fmt("%g", scope->GetChannelOffset(0, 0)));
	scope->SetChannelOffset(0, 0, 0);
	auto couplings = scope->GetAvailableCouplings(0);
	Check("couplings DC+AC", couplings.size() == 2);
	scope->SetChannelCoupling(0, OscilloscopeChannel::COUPLE_AC_1M);
	Check("coupling AC", scope->GetChannelCoupling(0) == OscilloscopeChannel::COUPLE_AC_1M);
	scope->SetChannelCoupling(0, OscilloscopeChannel::COUPLE_DC_1M);

	printf("== Trigger\n");
	auto trig = dynamic_cast<EdgeTrigger*>(scope->GetTrigger());
	Check("edge trigger pulled", trig != nullptr);
	if(!trig)
		return 1;
	trig->SetInput(0, StreamDescriptor(scope->GetOscilloscopeChannel(0), 0), true);
	trig->SetType(EdgeTrigger::EDGE_RISING);
	trig->SetLevel(0.1);
	scope->PushTrigger();
	scope->PullTrigger();
	trig = dynamic_cast<EdgeTrigger*>(scope->GetTrigger());
	Check("trigger level read back", fabs(trig->GetLevel() - 0.1) < 1e-4, Fmt("%g", trig->GetLevel()));
	Check("trigger edge read back", trig->GetType() == EdgeTrigger::EDGE_RISING);
	int64_t wantOffset = 5000LL * 1000000000LL;		//5 ms = half of 10k samples at 1 MS/s
	scope->SetTriggerOffset(wantOffset);
	Check("trigger offset 5 ms", llabs(scope->GetTriggerOffset() - wantOffset) <= 1000000000LL,
		Fmt("%.0f fs", scope->GetTriggerOffset()));

	printf("== Single capture\n");
	scope->StartSingleTrigger();
	bool ok = Capture(scope);
	Check("single capture", ok);
	if(ok)
	{
		auto w1 = Wfm(scope, 0);
		auto w2 = Wfm(scope, 1);
		Check("CHAN1 waveform present", w1 != nullptr);
		Check("CHAN2 waveform present", w2 != nullptr);
		if(w1 && w2)
		{
			w1->PrepareForCpuAccess();
			w2->PrepareForCpuAccess();
			Check("CHAN1 depth 10000", w1->size() == 10000, to_string(w1->size()));
			Check("timescale 1 us", w1->m_timescale == 1000000000LL, to_string(w1->m_timescale));

			float mn = 1e9, mx = -1e9;
			for(size_t i=0; i<w1->size(); i++)
			{
				mn = min(mn, w1->m_samples[i]);
				mx = max(mx, w1->m_samples[i]);
			}
			Check("CHAN1 ~1 Vpp sine around +0.1 V", fabs(mx - 0.6) < 0.05 && fabs(mn + 0.4) < 0.05, Fmt("min %.3f max %.3f", mn, mx));
			float mn2 = 1e9, mx2 = -1e9;
			for(size_t i=0; i<w2->size(); i++)
			{
				mn2 = min(mn2, w2->m_samples[i]);
				mx2 = max(mx2, w2->m_samples[i]);
			}
			Check("CHAN2 +/-1 V square", fabs(mx2 - 1) < 0.05 && fabs(mn2 + 1) < 0.05, Fmt("min %.3f max %.3f", mn2, mx2));

			//Find the rising 0.1 V crossing nearest the trigger offset, in displayed time
			int64_t trigT = scope->GetTriggerOffset();
			double best = 1e30;
			for(size_t i=1; i<w1->size(); i++)
			{
				float a = w1->m_samples[i-1], b = w1->m_samples[i];
				if(a < 0.1f && b >= 0.1f)
				{
					double frac = (0.1 - a) / (b - a);
					double t = ((i - 1) + frac) * w1->m_timescale + w1->m_triggerPhase;
					if(fabs(t - trigT) < fabs(best))
						best = t - trigT;
				}
			}
			//slope is ~3 mV/sample vs 4 mV rms noise, so allow a couple of samples of jitter
			Check("rising crossing lands on trigger marker", fabs(best) < 3 * w1->m_timescale,
				Fmt("error %.2f samples", best / w1->m_timescale));
		}
	}
	Check("single -> not armed", !scope->IsTriggerArmed());

	//Sub-sample alignment check on a steep edge (CHAN2 square, ~1 V/sample at 1 MS/s), where noise is
	//negligible. If the sign of m_triggerPhase were wrong the error would be ~2x trigger_frac, up to 2 samples.
	printf("== Sub-sample trigger alignment (CHAN2 square edge)\n");
	trig->SetInput(0, StreamDescriptor(scope->GetOscilloscopeChannel(1), 0), true);
	trig->SetType(EdgeTrigger::EDGE_RISING);
	trig->SetLevel(0.0);
	scope->PushTrigger();
	double worst = 0;
	int n = 0;
	for(int k=0; k<10; k++)
	{
		scope->StartSingleTrigger();
		if(!Capture(scope))
			continue;
		auto w = Wfm(scope, 1);
		w->PrepareForCpuAccess();
		int64_t trigT = scope->GetTriggerOffset();
		double best = 1e30;
		for(size_t i=1; i<w->size(); i++)
		{
			float a = w->m_samples[i-1], b = w->m_samples[i];
			if(a < 0 && b >= 0)
			{
				double t = ((i - 1) + (0 - a) / (b - a)) * w->m_timescale + w->m_triggerPhase;
				if(fabs(t - trigT) < fabs(best))
					best = t - trigT;
			}
		}
		worst = max(worst, fabs(best) / w->m_timescale);
		n++;
	}
	Check("10 captures on CHAN2 edge", n == 10, to_string(n));
	Check("crossing within 0.1 sample of trigger marker", worst < 0.1, Fmt("worst error %.3f samples", worst));
	trig->SetInput(0, StreamDescriptor(scope->GetOscilloscopeChannel(0), 0), true);
	trig->SetLevel(0.1);
	scope->PushTrigger();

	printf("== Continuous (RUN)\n");
	scope->Start();
	int got = 0;
	for(int i=0; i<5; i++)
		if(Capture(scope))
			got++;
	scope->Stop();
	Check("5 back-to-back captures in RUN mode", got == 5, to_string(got));

	printf("== Force trigger (NORMAL-equivalent: unreachable level)\n");
	trig->SetLevel(0.99);
	scope->PushTrigger();
	scope->ForceTrigger();
	Check("forced capture", Capture(scope));

	printf("\n%d passed, %d failed\n", g_pass, g_fail);
	scope = nullptr;
	ScopehalStaticCleanup();
	return g_fail ? 1 : 0;
}

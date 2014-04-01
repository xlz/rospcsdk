#include "ros/ros.h"
#include "std_msgs/String.h"
#include "roscpp/Empty.h"
#include "pxcvoice.h"
#include "util_capture.h"
#include "pcsdk/Grammar.h"
#include <boost/thread.hpp>
#include "voice_out.h"

#define PXC_ASSERT(expr) do { if (expr < 0) { ROS_FATAL(#expr); return -1;}} while(0)
#define PXC_CHECK(expr) do { if (expr < 0) ROS_ERROR(#expr);} while(0)

class recog_node: public PXCVoiceRecognition::Recognition::Handler, public PXCVoiceRecognition::Alert::Handler {
private:
	ros::NodeHandle node;
	ros::ServiceServer srvStop, srvDictate, srvGrammar;
	ros::Publisher pubRecog, pubAlert, pubSynth;
	ros::Subscriber subSynth;

	PXCSmartPtr<PXCSession> session;
	PXCSmartPtr<PXCVoiceRecognition> recogModule;
	PXCSmartPtr<PXCVoiceSynthesis> synthModule;
	PXCSmartPtr<UtilCapture> capture;
	VoiceOut *speaker;
	pxcUID currentGrammar;
	enum {
		STANDBY = 0,
		DICTATE,
		GRAMMAR
	} mode;
	boost::thread recogThread;
	bool speaking;

	void print_module_info(pxcUID cuid) {
		PXCSession::ImplDesc desc = {0}, desc2;
		desc.cuids[0] = cuid;
		PXC_CHECK(session->QueryImpl(&desc, 0, &desc2));
		ROS_INFO("%ls %d.%d", desc2.friendlyName, desc2.version.major, desc2.version.minor);
	}

	void speak(const std::string &sent) {
		std_msgs::StringPtr msg(new std_msgs::String);
		msg->data = sent;
		pubSynth.publish(msg);
	}
public:
	int start() {
		mode = STANDBY;
		speaking = false;

		PXC_ASSERT(PXCSession_Create(&session));

		PXCSession::ImplVersion version;
		PXC_ASSERT(session->QueryVersion(&version));
		ROS_INFO("Intel Perceptual Computing SDK %d.%d", version.major, version.minor);

		capture = new UtilCapture(session);
		if (capture == NULL) {
			ROS_FATAL("new UtilCapture");
			return -1;
		}

		PXC_ASSERT(session->CreateImpl<PXCVoiceRecognition>(&recogModule));
		PXC_ASSERT(session->CreateImpl<PXCVoiceSynthesis>(&synthModule));

		PXCVoiceRecognition::ProfileInfo pinfo;
		PXC_ASSERT(recogModule->QueryProfile(0, &pinfo));
		PXC_ASSERT(capture->LocateStreams(&pinfo.inputs));
		PXC_ASSERT(recogModule->SetProfile(&pinfo));
		PXC_ASSERT(recogModule->SubscribeRecognition(this));
		PXC_ASSERT(recogModule->SubscribeAlert(this));
		PXC_ASSERT(recogModule->SetDictation());
		print_module_info(PXCVoiceRecognition::CUID);

		PXCVoiceSynthesis::ProfileInfo pinfo2;
		PXC_ASSERT(synthModule->QueryProfile(0, &pinfo2));
		PXC_ASSERT(synthModule->SetProfile(&pinfo2));
		speaker = new VoiceOut(&pinfo2);
		if (speaker == NULL) {
			ROS_FATAL("new VoiceOut");
			return -1;
		}
		print_module_info(PXCVoiceSynthesis::CUID);

		srvStop = node.advertiseService("/pcsdk/recog/stop", &recog_node::srv_stop, this);
		srvDictate = node.advertiseService("/pcsdk/recog/dictate", &recog_node::srv_dictate, this);
		srvGrammar = node.advertiseService("/pcsdk/recog/grammar", &recog_node::srv_grammar, this);
		pubRecog = node.advertise<std_msgs::String>("/pcsdk/recog/speech", 10);
		pubAlert = node.advertise<std_msgs::String>("/pcsdk/recog/alert", 10);
		subSynth = node.subscribe("/pcsdk/synth", 2, &recog_node::sub_synth, this);
		pubSynth = node.advertise<std_msgs::String>("/pcsdk/synth", 5);

		recogThread = boost::thread(&recog_node::process_speech, this);

		speak("I can hear you now.");
		return 0;
	}
	void process_speech() {
		while (ros::ok()) {
			PXCSmartSPArray sps(2);
			PXCSmartPtr<PXCAudio> audio;
			PXC_CHECK(capture->ReadStreamAsync(&audio, &sps[0]));

			if (mode != STANDBY && !speaking)
				PXC_CHECK(recogModule->ProcessAudioAsync(audio, &sps[1]));

			sps.SynchronizeEx();
		}
	}
	void join() {
		recogThread.join();
	}
	bool srv_stop(roscpp::Empty::Request &req, roscpp::Empty::Response &res) {
		mode = STANDBY;
		PXC_CHECK(recogModule->ProcessAudioEOS());
		ROS_INFO("mode is standby");
		return true;
	}
	bool srv_dictate(roscpp::Empty::Request &req, roscpp::Empty::Response &res) {
		if (mode == GRAMMAR)
			PXC_CHECK(recogModule->ProcessAudioEOS());
		mode = DICTATE;
		PXC_CHECK(recogModule->SetDictation());
		ROS_INFO("mode is dication");
		return true;
	}
	bool srv_grammar(pcsdk::Grammar::Request &req, pcsdk::Grammar::Response &res) {
		if (mode == DICTATE)
			PXC_CHECK(recogModule->ProcessAudioEOS());
		mode = GRAMMAR;
		if (currentGrammar != 0)
			PXC_CHECK(recogModule->DeleteGrammar(currentGrammar));

		PXC_CHECK(recogModule->CreateGrammar(&currentGrammar));
		for (int i = 0; i < req.phrases.size(); i++) {
			std::wstring wide(req.phrases[i].begin(), req.phrases[i].end());
			PXC_CHECK(recogModule->AddGrammar(currentGrammar, i, (pxcCHAR *)wide.c_str()));
		}
		PXC_CHECK(recogModule->SetGrammar(currentGrammar));
		ROS_INFO("mode is grammar");
		return true;
	}
	void PXCAPI OnRecognized(PXCVoiceRecognition::Recognition *data) {
		std::wstring wide(data->dictation);
		std_msgs::String msg;
		msg.data.assign(wide.begin(), wide.end());
		pubRecog.publish(msg);
		speak(std::string("You said, ") + msg.data);
		ROS_INFO("heard #%d %ls", data->label, data->dictation);
	}
	void PXCAPI OnAlert(PXCVoiceRecognition::Alert *data) {
		std::stringstream ss;
		std_msgs::String msg;
		if (data->label & PXCVoiceRecognition::Alert::LABEL_VOLUME_HIGH)
			ss << "VOLUME_HIGH ";
		if (data->label & PXCVoiceRecognition::Alert::LABEL_VOLUME_LOW)
			ss << "VOLUME_LOW ";
		if (data->label & PXCVoiceRecognition::Alert::LABEL_SNR_LOW)
			ss << "SNR_LOW ";
		if (data->label & PXCVoiceRecognition::Alert::LABEL_SPEECH_UNRECOGNIZABLE)
			ss << "SPEECH_UNRECOGNIZABLE ";
		msg.data = ss.str();
		pubAlert.publish(msg);
		ROS_INFO("alert %s", msg.data.c_str());
	}
	void sub_synth(const std_msgs::String::ConstPtr& msg) {
		ROS_INFO("saying %s", msg->data.c_str());
		std::wstring wide(msg->data.begin(), msg->data.end());
		pxcUID id = 0;
		synthModule->QueueSentence((pxcCHAR *)wide.c_str(), wide.size(), &id);
		for (;;) {
			PXCSmartSP sp;
			PXCAudio *audio;
			int ret;
			ret = synthModule->ProcessAudioAsync(id, &audio, &sp);
			if (ret < 0)
				break;
			ret = sp->Synchronize();
			if (ret < 0)
				break;
			if (!speaking && mode != STANDBY)
				PXC_CHECK(recogModule->ProcessAudioEOS());
			speaking = true;
			speaker->RenderAudio(audio);
		}
		Sleep(1000);
		speaking = false;
	}
};

int main(int argc, char **argv) {
	if (setvbuf(stdout, NULL, _IONBF, 0)) {
		ROS_FATAL("setvbuf stdout");
		return 1;
	}
	if (setvbuf(stderr, NULL, _IONBF, 0)) {
		ROS_FATAL("setvbuf stderr");
		return 1;
	}

	ros::init(argc, argv, "pcsdk");

	recog_node recog;

	PXC_ASSERT(recog.start());

	ros::spin();

	recog.join();

	return 0;
}

#include <fcntl.h>
#include <limits.h>
#include <linux/soundcard.h>
#include <math.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <string>


/* 2 * 44100 (Hz) * 1 (sec) = 3528000 (bits) */

#define BUFSIZE (44100 * 2 * 1)


/// dspデバイス管理クラス
class Dsp
{
public:
        /// コンストラクタ。変数初期化のみ
        explicit Dsp(const std::string dev):
                device_name_(dev),
                fd_(-1)
        {
        };
        /// デバイスのオープン。失敗時には例外を投げる。
        void Open()
        {
                if ( ( fd_ = open(device_name_.c_str(), O_RDONLY ) ) == -1 ) {
                        throw std::runtime_error("device open error");
                }
        };
        /// DSPの設定を行う。失敗時には例外を投げる。
        void Setup(int fmt     = AFMT_S16_LE,
                   int freq    = 44100,
                   int channel = 1)
        {
                
                if ( ioctl( fd_, SOUND_PCM_SETFMT, &fmt ) == -1 ) {
                        throw std::runtime_error( "ioctl( SOUND_PCM_SETFMT )" );
                }
                
                if ( ioctl( fd_, SOUND_PCM_WRITE_CHANNELS, &channel ) == -1 ) {
                        throw std::runtime_error( "iotcl( SOUND_PCM_WRITE_CHANNELS )" );
                }
                
                if ( ioctl( fd_, SOUND_PCM_WRITE_RATE, &freq ) == -1 ) {
                        throw std::runtime_error( "iotcl( SOUND_PCM_WRITE_RATE )" );
                }
        };
        /// データの読み込み。1秒分読み込む。失敗時には例外を投げる。
        int Read(void *buf, size_t size)
        {
                int ret = read( fd_, buf, size );
                if (ret < 0)
                {
                        throw std::runtime_error("read error");
                }
                return ret;
        }
        /// 自動でcloseする
        ~Dsp()
        {
                if (fd_ >= 0)
                {
                        close(fd_);
                }
        }
private:
        /// オープンするデバイスパス。"/dev/dsp"など
        std::string device_name_;
        /// ファイルディスクリプタ
        int fd_;
};


/// 音声認識器もどき
/// 音の大きさだけで判断する。
class FakeRecognizer
{
public:
        /// dspの参照を取るコンストラクタ
        explicit FakeRecognizer(Dsp &dsp):
                dsp_(dsp),
                threshould_(500),
                fake_result_("")
        {
        };
        /// キャリブレーションコールバック
        /// １秒記録して1.2倍したものをスレッショルドとする。
        bool Calib(std_srvs::Empty::Request &req,
                   std_srvs::Empty::Response &res)
        {
                const double average = GetAverageVolume();
                ROS_INFO("calib %f -> %f", threshould_, average * 1.2);
                threshould_ = average * 1.2;
                return true;
        };
        /// 音の大きさの１秒の平均値
        double GetAverageVolume()
        {
                short  buf[ BUFSIZE / sizeof(short) ];
                dsp_.Read(buf, BUFSIZE );
                double average = (double)buf[0];
                
                for (uint32_t i = 1; i < BUFSIZE / sizeof(short); i ++)
                {
                        average = ((average * i) + abs(buf[i]))/(i+1);
                }
                return average;
        };
        /// 認識させる（音量のみ）
        const std::string Recog()
        {
                const double average = GetAverageVolume();
                ROS_INFO("%f [%f]", average, threshould_);
                if (average > threshould_)
                {
                        //ROS_INFO("%f > %f", average, threshould);
                        return fake_result_;
                }
                return std::string("");;
        };
        
        void SetThreshould(const double thre)
        {
                threshould_ = thre;
        };
        void SetThreshould(const std_msgs::Float64::ConstPtr &thre)
        {
                threshould_ = thre->data;
        };
        double GetThreshould()
        {
                return threshould_;
        };
        void SetFakeResult(const std::string &result)
        {
                fake_result_ = result;
        };
        void SetFakeResult(const std_msgs::String::ConstPtr &result)
        {
                fake_result_ = result->data;
        };
private:
        /// DSPへの参照(本当はshared_ptrのほうがいい)
        Dsp &dsp_;
        /// 認識したことにする音量の閾値
        double threshould_;
        /// 認識したとする文字列
        std::string fake_result_;
};


int main(int argc, char *argv[])
{
        double threshould = 500.0;
        std_msgs::String str;

        ros::init(argc, argv, "fake_voice_recog");

        ros::NodeHandle node;
        ros::NodeHandle local_node("~");

        ros::Publisher pub = node.advertise<std_msgs::String>("recog", 10);

        std::string device_name;
        local_node.param<std::string>("device", device_name, "/dev/dsp");

        Dsp dsp(device_name);
        try
        {
                dsp.Open();
                dsp.Setup();
        }
        catch(std::runtime_error &e)
        {
                std::cerr << e.what() << std::endl;
                exit(1);
        }

        FakeRecognizer recog(dsp);
        recog.SetFakeResult("hoge");
  
        ros::ServiceServer service = local_node.advertiseService("calib", &FakeRecognizer::Calib, &recog);
        ros::Subscriber result_sub = local_node.subscribe("fake_result", 10, &FakeRecognizer::SetFakeResult, &recog);
        ros::Subscriber thre_sub = local_node.subscribe("threshould", 10, &FakeRecognizer::SetThreshould, &recog);

        local_node.getParam("threshould",threshould);
        recog.SetThreshould(threshould);

        ros::Rate r(100);
        while(node.ok())
        {
                try
                {
                        r.sleep();
                        ros::spinOnce();
                        std::string result = recog.Recog();
                        if (result != "")
                        {
                                ROS_INFO("publish! %s", result.c_str());
                                str.data = result;
                                pub.publish(str);
                        }
                }
                catch(std::runtime_error &e)
                {
                        std::cerr << e.what() << std::endl;
                }
        }
  
        return 0;
}




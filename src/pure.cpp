#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>

using namespace std;

struct Point {
    double x, y, theta;
};

class PIDController {
public:
    PIDController(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}

    double compute(double error) {
        integral_ += error;
        double derivative = error - prev_error_;
        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void reset() {
        prev_error_ = 0;
        integral_ = 0;
    }

    void set_gains(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

private:
    double kp_, ki_, kd_;
    double prev_error_, integral_;
};

class PIDNavigator : public rclcpp::Node {
public:
    PIDNavigator() : Node("pid"),
                     current_x_(0), current_y_(0), current_theta_(0), target_index_(0),
                     is_first_target_(true),  // 初期化順を修正
                     linear_pid_(0.0, 0.0, 0.0),  // 初期化時にデフォルト値を設定
                     angular_pid_(0.0, 0.0, 0.0) {  // 初期化時にデフォルト値を設定
        // パラメータの宣言
        this->declare_parameter<double>("linear_kp", 1.0);
        this->declare_parameter<double>("linear_ki", 0.0);
        this->declare_parameter<double>("linear_kd", 0.0);
        this->declare_parameter<double>("angular_kp", 0.05);
        this->declare_parameter<double>("angular_ki", 0.0);
        this->declare_parameter<double>("angular_kd", 0.0);
        this->declare_parameter<double>("max_linear_velocity", 0.5);
        this->declare_parameter<double>("max_angular_velocity", 1.0);
        this->declare_parameter<double>("max_accel", 1.0);
        this->declare_parameter<double>("arrival_threshold", 0.1);
        this->declare_parameter<double>("theta_threshold", 0.1);
        this->declare_parameter<int>("kokuban_t", 1000);
        this->declare_parameter<int>("turn_t", 100);
        this->declare_parameter<int>("launcher_1_t", 2000);
        this->declare_parameter<int>("ball_t", 1000);
        this->declare_parameter<int>("launcher_2_t", 100000);
        this->declare_parameter("field_color", "red");
        this->declare_parameter("red_csv", "/home/yuki/ros2_ws/src/uni_tes/path/red.csv");
        this->declare_parameter("blue_csv", "/home/yuki/ros2_ws/src/uni_tes/path/blue.csv");
        this->declare_parameter<double>("stop_threshold", 0.05);
        this->declare_parameter<double>("look_ahead", 0.5);


        this->get_parameter("linear_kp", linear_kp);
        this->get_parameter("linear_ki", linear_ki);
        this->get_parameter("linear_kd", linear_kd);
        this->get_parameter("angular_kp", angular_kp);
        this->get_parameter("angular_ki", angular_ki);
        this->get_parameter("angular_kd", angular_kd);
        
        cout <<"angular_kp: " << angular_kp << endl;
        linear_pid_.set_gains(linear_kp, linear_ki, linear_kd);
        angular_pid_.set_gains(angular_kp, angular_ki, angular_kd);

        this->get_parameter("max_linear_velocity", max_linear_vel_);
        this->get_parameter("max_angular_velocity", max_angular_vel_);
        this->get_parameter("max_accel", max_accel_);
        this->get_parameter("arrival_threshold", arrival_threshold_);
        this->get_parameter("theta_threshold", theta_threshold_);
        this->get_parameter("kokuban_t", t1);
        this->get_parameter("turn_t", t2);
        this->get_parameter("launcher_1_t", t3);
        this->get_parameter("ball_t", t4);
        this->get_parameter("launcher_2_t", t5);
        this->get_parameter("stop_threshold", stop_threshold_);
        this->get_parameter("look_ahead", look_ahead_);

        field_color_ = this->get_parameter("field_color").as_string();
        red_csv = this->get_parameter("red_csv").as_string();
        blue_csv = this->get_parameter("blue_csv").as_string();

        wait_time = {t1, t2, t3, t4, t5};

        if(field_color_ == "red") load_waypoints(red_csv);
        else if(field_color_ == "blue") load_waypoints(blue_csv);  

        generateBezierPath();


        RCLCPP_INFO(this->get_logger(), "field_color: %s", field_color_.c_str());

        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/pose", 10, std::bind(&PIDNavigator::pose_callback, this, std::placeholders::_1));
        start_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/start", 10, std::bind(&PIDNavigator::start_callback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/pid_cmd_vel", 10);
        state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/state", 10);
        launcher1_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/launcher1", 10);
        kokuban_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/kokuban", 10);
        ball_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/ball", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(dt), std::bind(&PIDNavigator::control_loop, this));

    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr launcher1_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kokuban_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ball_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    vector<Point> waypoints_;
    vector<vector<Point>> paths;
    double current_x_, current_y_, current_theta_;
    size_t target_index_;
    size_t current_index_ = 0;
    double look_ahead_ ;
    bool is_first_target_;
    PIDController linear_pid_;
    PIDController angular_pid_;
    // パラメータの読み込み
    double linear_kp, linear_ki, linear_kd;
    double angular_kp, angular_ki, angular_kd;
    double max_linear_vel_, max_angular_vel_, max_accel_;
    double arrival_threshold_, theta_threshold_;
    bool utikiri_;
    double stop_threshold_;

    int t1,t2,t3,t4,t5;
    vector<int> wait_time;
    string field_color_,red_csv,blue_csv;
    bool start_flag = false;
    int dt = 10; //(ms)
    bool time_flag =true;
    bool stop =false;
    chrono::steady_clock::time_point start_time; // クラスメンバとして宣言




    void pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_theta_ = msg->theta;
    }

    void start_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        start_flag = msg->data;
    }

    void control_loop() {
        if (target_index_ >= waypoints_.size()) {
            auto cmd = geometry_msgs::msg::Twist();
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            cout<<"Reached the goal!"<<endl;
            return;
        }

        if(!start_flag) return;

        if(time_flag){
            start_time = chrono::steady_clock::now();
            time_flag = false;
        }

        while (current_index_ < paths[target_index_].size() - 1 &&
               sqrt(pow(current_x_ - paths[target_index_][current_index_].x, 2) + pow(current_y_ - paths[target_index_][current_index_].y, 2)) < look_ahead_) {
            ++current_index_;
        }

        
        const auto& target = paths[target_index_][current_index_];
        double dx = target.x - current_x_;
        double dy = target.y - current_y_;
        double distance = sqrt(dx * dx + dy * dy);
        double angle_error = M_PI*target.theta/180 - current_theta_;

        double distance_to_goal = sqrt(pow(current_x_ - paths[target_index_].back().x, 2) + pow(current_y_ - paths[target_index_].back().y, 2));
        double linear_vel = linear_pid_.compute(distance_to_goal);
        double angular_vel = angular_pid_.compute(angle_error);

        auto current_time = chrono::steady_clock::now();
        auto elapsed_time = chrono::duration_cast<chrono::seconds>(current_time - start_time).count();
        // 速度制限
        linear_vel = min({linear_vel, max_linear_vel_, max_accel_ * elapsed_time});
        angular_vel = max(min(angular_vel, max_angular_vel_), -max_angular_vel_);

        double v_x = dx * cos(current_theta_) + dy * sin(current_theta_);
        double v_y = -dx * sin(current_theta_) + dy * cos(current_theta_);

        //target=0の終わりでは |dy|が閾値以下で停止
        //target=1の終わりでは |dx|が閾値以下で停止
        //target=2の終わりでは そのまま
        //target=3の終わりでは |dy|が閾値以下で停止
        //target=4の終わりでは そのまま

        if(target_index_==0 && abs(dy)<stop_threshold_) stop = true;
        if(target_index_==1 && abs(dx)<stop_threshold_) stop = true;
        if(target_index_==2 && distance < arrival_threshold_ && abs(angle_error) < theta_threshold_) stop =true;
        if(target_index_==3 && abs(dy)<stop_threshold_) stop = true;
        if(target_index_==4 && distance < arrival_threshold_ && abs(angle_error) < theta_threshold_)stop =true;
        if (stop) {

            //stateが2,4の終わりのときに射出を送る
            if(target_index_ == 2 || target_index_ == 4){
                auto shoot_cmd = std_msgs::msg::Bool();
                shoot_cmd.data = true;
                launcher1_publisher_->publish(shoot_cmd);
            }
            //stateが0の終わりに黒板回収
            if(target_index_ == 0){
                auto kokuban_cmd = std_msgs::msg::Bool();
                kokuban_cmd.data = true;
                kokuban_publisher_->publish(kokuban_cmd);
            }
            //stateが3の終わりにボール回収
            if(target_index_ == 3){
                auto ball_cmd = std_msgs::msg::Bool();
                ball_cmd.data = true;
                ball_publisher_->publish(ball_cmd);
            }

            target_index_++;
            cout<<"target_index: "<<target_index_<<endl;
            auto state_msg = std_msgs::msg::Int32();
            state_msg.data = target_index_;
            //state_publisher_->publish(state_msg);

            linear_pid_.reset();
            angular_pid_.reset();

            current_index_ = 0;

            auto stop_cmd = geometry_msgs::msg::Twist();
            stop_cmd.linear.x = 0.0;
            stop_cmd.linear.y = 0.0;
            stop_cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(stop_cmd);

            this_thread::sleep_for(chrono::milliseconds(wait_time[target_index_-1]));
            state_publisher_->publish(state_msg);//停止後少し動くのでここでstateを送信(直っているか後で確認)
            time_flag = true;
            stop = false;
        }

        auto cmd = geometry_msgs::msg::Twist();
        double norm_factor = sqrt(v_x * v_x + v_y * v_y);
        if (norm_factor > 0) {
            cmd.linear.x = linear_vel * (v_x / norm_factor);
            cmd.linear.y = linear_vel * (v_y / norm_factor);
        } else {
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
        }
        cmd.angular.z = angular_vel;
        cmd_vel_pub_->publish(cmd);
    }

    void load_waypoints(const string& filename) {
        ifstream file(filename);
        string line;
        while (getline(file, line)) {
            stringstream ss(line);
            Point wp;
            char comma;
            if (ss >> wp.x >> comma >> wp.y >> comma >> wp.theta) {
                waypoints_.push_back(wp);
            }
        }
    }

    //csvファイル内に始点がないと直線生成できないことに注意(pidより点が一個多い)

    void generateBezierPath() {
    int points_per_segment = 1000;
    
    // paths をクリアして新しいパスを格納
    paths.clear();

    // 各 path について処理
    for (size_t i = 0; i < waypoints_.size() - 1; i++) {
        vector<Point> p_path;  // 1つの経路のデータを格納

        // 各セグメントを処理
        //打ち切り制御をするため、各経路の最後にstop_threshold分の直線を追加
        double ex_x =0;
        double ex_y =0;
        if(i==0)
        {
            ex_x=0;
            ex_y=stop_threshold_;
        }
        if(i==1)
        {
            if(field_color_=="blue") ex_x=stop_threshold_;
            else ex_x=-stop_threshold_;
            ex_y=0;
        }
        if(i==2)
        {
            ex_x=0;
            ex_y=-stop_threshold_;
        }
        if(i==3)
        {
            ex_x=0;
            ex_y=stop_threshold_;
        }
        if(i==4)
        {
            ex_x=0;
            ex_y=-stop_threshold_;
        }

        for (int j = 0; j < points_per_segment; j++) {  // `<=` にして最終点も含める
            double t = static_cast<double>(j) / points_per_segment;
            double path_x = (1 - t) * waypoints_[i].x + t * (waypoints_[i + 1].x+ex_x);
            double path_y = (1 - t) * waypoints_[i].y + t * (waypoints_[i + 1].y+ex_y);   
            double path_theta = waypoints_[i+1].theta;
            p_path.push_back({path_x, path_y, path_theta});
        }
        paths.push_back(p_path);  // 完成した経路を保存
    }
}
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

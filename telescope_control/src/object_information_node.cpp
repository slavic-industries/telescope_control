#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <curl/curl.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "telescope_interfaces/srv/query_object.hpp"
#include "nlohmann/json.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;

class ObjectInformationNode : public rclcpp::Node
{
public:
  ObjectInformationNode() : Node("object_information_node")
  {
    // Create a service to handle queries for object information
    service_ = this->create_service<telescope_interfaces::srv::QueryObject>(
        "query_object", std::bind(&ObjectInformationNode::handle_query, this, std::placeholders::_1, std::placeholders::_2));

    // Fetch and save the object database
    fetch_and_save_database();
  }

private:
  void fetch_and_save_database()
  {
    // URL of the SIMBAD or NASA database (this is a placeholder URL)
    std::string url = "http://example.com/astronomical_objects.json";

    CURL *curl;
    CURLcode res;
    std::string readBuffer;

    curl = curl_easy_init();
    if (curl)
    {
      curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
      res = curl_easy_perform(curl);
      curl_easy_cleanup(curl);

      if (res != CURLE_OK)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to fetch database: %s", curl_easy_strerror(res));
        return;
      }
    }

    // Save the database to a file
    std::ofstream file("object_database.json");
    if (file.is_open())
    {
      file << readBuffer;
      file.close();
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to open file to save database");
    }
  }

  static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
  {
    ((std::string *)userp)->append((char *)contents, size * nmemb);
    return size * nmemb;
  }

  void handle_query(
      const std::shared_ptr<telescope_interfaces::srv::QueryObject::Request> request,
      std::shared_ptr<telescope_interfaces::srv::QueryObject::Response> response)
  {
    std::ifstream file("object_database.json");
    if (!file.is_open())
    {
      response->object_info = "Database not found";
      return;
    }

    json object_database;
    file >> object_database;

    for (const auto &item : object_database)
    {
      if (item["name"] == request->object_name)
      {
        std::stringstream ss;
        ss << "Object: " << item["name"] << "\n"
           << "Type: " << item["type"] << "\n"
           << "Distance: " << item["distance"] << "\n"
           << "Constellation: " << item["constellation"] << "\n"
           << "Magnitude: " << item["magnitude"];
        response->object_info = ss.str();
        return;
      }
    }

    response->object_info = "Object information for '" + request->object_name + "' not found.";
  }

  rclcpp::Service<telescope_interfaces::srv::QueryObject>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectInformationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

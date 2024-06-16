#ifndef RTLBS_SERIALIZABLE
#define RTLBS_SERIALIZABLE


#include "rtlbs.h"
#include "utility/define.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/prettywriter.h"



class Serializable {
public:
	virtual ~Serializable() {}
	//将对象进行序列化 得到字符串
	virtual void Serialize(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) const { };
	//将json文件中读取得到的value转换为对象
	virtual bool Deserialize(const rapidjson::Value& value) { return false; };
};


template <typename T>
void SerializeEnum(const T& item, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) {
	writer.Int(static_cast<int>(item));
}

template <typename T>
bool DeserializeEnum(T& item, const rapidjson::Value& value) {
	if (value.IsInt()) {
		item = static_cast<T>(value.GetInt());
		return true;
	}
	return false;

}

template <typename T>
void SerializeArray(const std::vector<T>& items, rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer) {
	writer.StartArray();
	for (auto item : items) {
		if constexpr (std::is_arithmetic_v<T> || std::is_same_v<T, std::string> || std::is_same_v<T, bool>) {
			if constexpr (std::is_same_v<T, std::string>) {
				// for string, serialize as string
				writer.String(item.c_str());
				continue;
			}
			if constexpr (std::is_same_v<T, bool>) {
				// for bool, serialize as bool
				writer.Bool(item);
				continue;
			}
			// for arithmetic types, string and bool, simply serialize as is
			if constexpr (std::is_floating_point_v<T>) {
				writer.Double(item);
				continue;
			}
			if constexpr (std::is_unsigned_v<T>) {
				writer.Uint(item);
				continue;
			}
			if constexpr (std::is_same_v<T, int64_t>) {
				writer.Int64(item);
				continue;
			}
		}
		else if constexpr (std::is_enum_v<T>) {
			// for enum types, serialize as integer value
			writer.Int(static_cast<int>(item));
		}
		else {
			// for user-defined types, call the Serialize() method of the object
			item.Serialize(writer);
		}
	}
	writer.EndArray();
}

template <typename T>
bool DeserializeArray(std::vector<T>& items, const rapidjson::Value& value) {
	//初始化时放弃items中的所有元素，防止二次赋值
	items.clear();
	for (rapidjson::SizeType i = 0; i < value.Size(); i++) {
		T item;
		if constexpr (std::is_arithmetic_v<T>) {					//基本算术类型
			if constexpr (std::is_same_v<T, int>) {
				item = value[i].GetInt();
			}
			else if constexpr (std::is_same_v<T, unsigned>) {
				item = value[i].GetUint();
			}
			else if constexpr (std::is_same_v<T, int64_t>) {
				item = value[i].GetInt64();
			}
			else if constexpr (std::is_same_v<T, float>) {
				item = value[i].GetFloat();
			}
			else if constexpr (std::is_same_v<T, double>) {
				item = value[i].GetDouble();
			}
			else if constexpr (std::is_same_v<T, bool>) {
				item = value[i].GetBool();
			}
		}
		else if constexpr (std::is_enum_v<T>) {						//枚举类型
			// for enum types, deserialize as integer value and cast to enum
			item = static_cast<T>(value[i].GetInt());
		}
		else if constexpr (std::is_same_v<T, std::string>) {
			item = value[i].GetString();
		}
		else {														//其他类型
			// for user-defined types, call the Deserialize() method of the object
			if (!item.Deserialize(value[i])) {
				return false;
			}
		}
		items.push_back(item);
	}
	return true;
}
#endif

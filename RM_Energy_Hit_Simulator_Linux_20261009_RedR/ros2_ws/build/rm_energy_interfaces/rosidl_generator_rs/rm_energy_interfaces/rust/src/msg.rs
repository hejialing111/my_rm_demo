#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to rm_energy_interfaces__msg__GimbalCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GimbalCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_rad: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pitch_rad: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fire: bool,

}



impl Default for GimbalCommand {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::GimbalCommand::default())
  }
}

impl rosidl_runtime_rs::Message for GimbalCommand {
  type RmwMsg = super::msg::rmw::GimbalCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        yaw_rad: msg.yaw_rad,
        pitch_rad: msg.pitch_rad,
        fire: msg.fire,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      yaw_rad: msg.yaw_rad,
      pitch_rad: msg.pitch_rad,
      fire: msg.fire,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      yaw_rad: msg.yaw_rad,
      pitch_rad: msg.pitch_rad,
      fire: msg.fire,
    }
  }
}



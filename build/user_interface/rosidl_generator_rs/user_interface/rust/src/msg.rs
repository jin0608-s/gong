#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to user_interface__msg__UserInt

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UserInt {

    // This member is not documented.
    #[allow(missing_docs)]
    pub user_int: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub user_int2: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub user_int3: i32,

}



impl Default for UserInt {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::UserInt::default())
  }
}

impl rosidl_runtime_rs::Message for UserInt {
  type RmwMsg = super::msg::rmw::UserInt;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        user_int: msg.user_int,
        user_int2: msg.user_int2,
        user_int3: msg.user_int3,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      user_int: msg.user_int,
      user_int2: msg.user_int2,
      user_int3: msg.user_int3,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      user_int: msg.user_int,
      user_int2: msg.user_int2,
      user_int3: msg.user_int3,
    }
  }
}



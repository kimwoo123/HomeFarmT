const { Schedule } = require("../../models/index")
const crypto = require('crypto')

module.exports = {
  Query: {
    allUser: async () =>{
      const getUsers = await User.findAll();
      return getUsers;
  },
    findUser: async (_, { email }) => {
      const oneUser = await User.findOne({ where: {email: email}});
      return oneUser
    }
  }
};

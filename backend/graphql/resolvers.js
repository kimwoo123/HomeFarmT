const { User } = require("../models/index");

module.exports = {
  Query: {
    me: async (_, __, { dataSources }) =>
      dataSources.userAPI.findOrCreateUser(),
  },
  Mutation: {
    signUp: async (_, { userid, email }) => {
      const newUser = await User.create({ 
        userid,
        email
      })
      const user = await User.findOne( {where: { userid: userid }})
      return user
    },
    deleteUser: async (_, { email }) => {
      const oldUser = await User.delete({ where: { email: email }})
      const user = await User.findOne({ where: { email: email }})
      return user
    }
  },
};

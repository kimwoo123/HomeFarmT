const { User } = require("../models/index");

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
},
  Mutation: {
    signUp: async (_, { email, password }) => {
      const newUser = await User.create({ 
        password,
        email
      })
      const user = await User.findOne( {where: { email: email }})
      return user
    },
    deleteUser: async (_, { email }) => {
      const oldUser = await User.delete({ where: { email: email }})
      const user = await User.findOne({ where: { email: email }})
      return user
    }
  }
};

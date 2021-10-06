const { User } = require("../models/index")
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
    },
  },
  Mutation: {
    signUp: async (_, { email, password }) => {
      // const hashed = crypto.createHmac('sha256', email).update(password).digest('hex');
      const [userInfo, created] = await User.findOrCreate({ where: { email: email, password: password }})
      if (created) {
        console.log('signup done')
        return userInfo
      } else {
        console.log('user already exist')
        return userInfo
      }
      return userInfo
    },
    login: async (_, { email, password }) => {
      // const hashed = crypto.createHmac('sha256', email).update(password).digest('hex');
      const userCheck = await User.findOne({ where: { email: email, password: password}})
      if (userCheck.email === '') {
        console.log('none')
        throw new Error('user not exist')
      } else {
        const savedUser = await User.findOne({ where: { email }});
        return savedUser;
      }
    },
    deleteUser: async (_, { email }) => {
      const oldUser = await 
      delete({ where: { email: email }})
      const user = await User.findOne({ where: { email: email }})
      return user
    }
  }
};

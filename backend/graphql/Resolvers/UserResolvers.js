const { User } = require("../../models/index")
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
},
  Mutation: {
    signUp: async (_, { email, password }) => {
      // const hashed = crypto.createHmac('sha256', email).update(password).digest('hex');
      const [userInfo, created] = await User.findOrCreate({ where: { email: email, password: password }})
      if (created) {
        return userInfo
      } else {
        throw new Error('user already exist')
      }
    },
    login: async (_, { email, password }) => {
      // const hashed = crypto.createHmac('sha256', email).update(password).digest('hex');
      const userCheck = await User.findOne({ where: { email: email }})
      if (userCheck.email === '') {
        console.log(email, password)
        throw new Error('user not exist')
      } else {
        console.log(email, password)
        const savedUser = await User.findOne({ where: { email }});
        return savedUser;
      }
    },
    deleteUser: async (_, { email }) => {
      const oldUser = await User.delete({ where: { email: email }})
      const user = await User.findOne({ where: { email: email }})
      return user
    }
  }
};

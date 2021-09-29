const { User } = require("../../models/index")
const jwt = require("jsonwebtoken");
const crypto = require('crypto')

function isEmail(email) {
  let validate = /\S+@\S+\.\S+/
  return validate.test(email)
}

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
      console.log(isEmail(email))
      if (!isEmail(email)) {
        throw new Error('이메일의 형식이 아닙니다')
      }
      const hashedEmail = crypto.createHash('sha512').update(email).digest('base64')
      const userCheck = await User.findOne({ where: { email: hashedEmail }})
      if (userCheck && userCheck.email === hashedEmail) {
        throw new Error('email already exist')
      }

    const salt = crypto.randomBytes(26).toString('base64');
    const scryptPassword = crypto.scryptSync(password, salt, 26)
    const hashedPassword = scryptPassword.toString('hex')
    const result = await User.create({ 
      email: hashedEmail,
      password: hashedPassword,
      hashid: salt
    })
      console.log(hashedPassword)
      return result
    },
    login: async (_, { email, password }) => {
      const hashedEmail = crypto.createHash('sha512').update(email).digest('base64')
      const userCheck = await User.findOne({ where: { email: hashedEmail }})
      if (!userCheck) {
        throw new Error('user not exist')
      }
      const salt = await userCheck.hashid.toString('base64')
      const scryptPassword = crypto.scryptSync(password, salt, 26)
      const hashedPassword = scryptPassword.toString('hex')

      if (userCheck.password !== hashedPassword) {
        throw new Error('Password Error')
      }
      let token = jwt.sign({ hashedEmail, hashedPassword }, process.env.SECRET_KEY, { expiresIn: '5m'})
      const result = ({ 
        email: hashedEmail,
        password: hashedPassword,
        message: 'token!',
        token: token
      })
      return result;
    },
    deleteUser: async (_, { email }) => {
      const oldUser = await User.delete({ where: { email: email }})
      const user = await User.findOne({ where: { email: email }})
      return user
    }
  }
};

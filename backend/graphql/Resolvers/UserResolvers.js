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
    findUser: async (_, args, context) => {
      if (!context) {
        throw new Error('로그인이 필요합니다')
      }
      const user = await User.findOne({ where: { email: context.hashedEmail }})
      console.log(user)
      return user
    }
},
  Mutation: {
    signUp: async (_, { email, password }) => {
      if (!isEmail(email)) {
        throw new Error('이메일의 형식이 아닙니다')
      }
      const hashedEmail = crypto.createHash('sha512').update(email).digest('base64')
      const userCheck = await User.findOne({ where: { email: hashedEmail }})
      if (userCheck && userCheck.email === hashedEmail) {
        throw new Error('이미 존재하는 이메일입니다.')
      }

    const salt = crypto.randomBytes(26).toString('base64');
    const scryptPassword = crypto.scryptSync(password, salt, 26)
    const hashedPassword = scryptPassword.toString('hex')
    await User.create({ 
      email: hashedEmail,
      password: hashedPassword,
      hashid: salt
    })
    let token = jwt.sign({ hashedEmail, hashedPassword }, process.env.SECRET_KEY, { expiresIn: '100m'})
    const result = ({ 
      email: hashedEmail,
      token: token
    })
      return result
    },
    login: async (_, { email, password }) => {
      const hashedEmail = crypto.createHash('sha512').update(email).digest('base64')
      const userCheck = await User.findOne({ where: { email: hashedEmail }})
      if (!userCheck) {
        throw new Error('계정이 등록되어 있지 않습니다')
      }
      const salt = await userCheck.hashid.toString('base64')
      const scryptPassword = crypto.scryptSync(password, salt, 26)
      const hashedPassword = scryptPassword.toString('hex')

      if (userCheck.password !== hashedPassword) {
        throw new Error('비밀번호가 다릅니다')
      }
      let token = jwt.sign({ hashedEmail, hashedPassword }, process.env.SECRET_KEY, { expiresIn: '100m'})
      const result = ({ 
        email: hashedEmail,
        region: userCheck.region,
        token: token
      })
      return result;
    },
    updateUser: async(_, { email, password, turtlebot, region }, context) => {
      if (!context) {
        throw new Error('로그인이 필요합니다')
      }
      let hashedPassword
      let salt
      if (password) {
        salt = crypto.randomBytes(26).toString('base64');
        const scryptPassword = crypto.scryptSync(password, salt, 26)
        hashedPassword = scryptPassword.toString('hex')
      } else {
        const user = await User.findOne({ where: { email: context.hashedEmail }})
        salt = user.hashid
        hashedPassword = user.password
      }
      const userInfo = await User.update({password: hashedPassword, hashid: salt, turtlebot, region },
        { where: { 
          email: context.hashedEmail 
        }
      })
      return userInfo
    },
    deleteUser: async (_, { email }) => {
      const oldUser = await User.delete({ where: { email: email }})
      const user = await User.findOne({ where: { email: email }})
      return user
    }
  }
};

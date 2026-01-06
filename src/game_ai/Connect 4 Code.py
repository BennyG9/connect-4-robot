import random as rand
import numpy as np
import time
from concurrent.futures import ProcessPoolExecutor
import matplotlib.pyplot as plt

ROWS = 6
COLS = 7

ROW_MASKS = [sum(1 << (c*COLS + r) for c in range(COLS)) for r in range(ROWS+1)]
COL_MASKS = [0b1111111 << 7*c for c in range(COLS)]

def print_board(player, opponent):
    b = ''
    for r in reversed(range(ROWS+1)):
        for c in range(COLS):
            i = c * COLS + r
            bit = 1 << i

            if(bit & player > 0):
                b += "X "
            elif(bit & opponent > 0):
                b += "O "
            else:
                if(r != 6):
                    b += ". "
                else:
                    b += "_ "
        b += '\n'
    print(b)
    pass

def move_available(c, player_board, opponent_board):
    choice_mask = 0b0100000 << ((ROWS+1) * c)
    return (player_board | opponent_board) & choice_mask == 0

def get_move(c, player_board, opponent_board):
    if(not move_available(c, player_board, opponent_board)):
        raise ValueError('MOVE UNAVAILABLE')
    all_moves = (player_board | opponent_board)
    move = (all_moves & COL_MASKS[c]) + (1 << (ROWS+1)*c)
    return move

def moves_available(player_board, opponent_board):
    moves = []
    all_moves = player_board | opponent_board
    top_row = all_moves & ROW_MASKS[5]
    for c in rand.sample(range(COLS), COLS):
        if(top_row & 0b0100000 << ((ROWS+1) * c) == 0):
            moves.append(c)
    return moves

def check_win(player):
    cols = player & (player << 1) & (player << 2) & (player << 3)
    rows = player & (player << 7) & (player << 14) & (player << 21)
    u_diag = player & (player << 8) & (player << 16) & (player << 24)
    d_diag = player & (player << 6) & (player << 12) & (player << 18)
    #print_board(cols, 0)
    #print_board(rows, 0)
    #print_board(u_diag, 0)
    #print_board(d_diag, 0)
    return cols | rows | u_diag | d_diag > 0

def num_wins(player):
    cols = player & (player << 1) & (player << 2) & (player << 3)
    rows = player & (player << 7) & (player << 14) & (player << 21)
    u_diag = player & (player << 8) & (player << 16) & (player << 24)
    d_diag = player & (player << 6) & (player << 12) & (player << 18)
    return cols.bit_count() + rows.bit_count() + u_diag.bit_count() + d_diag.bit_count()

def wins(player):
    cols = player & (player << 1) & (player << 2) & (player << 3)
    rows = player & (player << 7) & (player << 14) & (player << 21)
    u_diag = player & (player << 8) & (player << 16) & (player << 24)
    d_diag = player & (player << 6) & (player << 12) & (player << 18)
    return cols | rows | u_diag | d_diag

def play_game(pl1_query, pl2_query):
    pl1 = 0
    pl2 = 0
    turn = (-1) ** (rand.randint(0,1))
    while(True):

        if(turn == 1):
            c = pl1_query(pl1, pl2)
            pl1 += get_move(c, pl1, pl2)
        else:
            c = pl2_query(pl2, pl1)
            pl2 += get_move(c, pl2, pl1)

        print_board(pl1, pl2)
        print(fill_feat_vec(pl1,pl2))

        #check for pl1 win
        if(turn == 1 and check_win(pl1)):
            print("PLAYER 1 WINS")
            return 
        #check for pl2 win
        elif(check_win(pl2)):
            print("PLAYER 2 WINS")
            return 
        #check for draw
        elif((pl1 | pl2).bit_count() == ROWS*COLS):
            print("DRAW")
            return 

        turn *= -1
        pass
    pass

def simulate_game(model1, model2, depth, turn):
    pl1 = 0
    pl2 = 0
    #turn = (-1) ** (rand.randint(0,1))
    #print(f"T: {turn}")
    while(True):
        if(turn == 1):
            c = minimax(pl1, pl2, model1, depth)
            #print(c)
            pl1 += get_move(c[1], pl1, pl2)
        else:
            c = minimax(pl2, pl1, model2, depth)
            #print(c)
            pl2 += get_move(c[1], pl2, pl1)

        #print_board(pl1, pl2)
        #print(fill_feat_vec(pl1,pl2))

        #check for pl1 win
        if(turn == 1 and check_win(pl1)):
            #print("PLAYER 1 WINS")
            return 1
        #check for pl2 win
        elif(check_win(pl2)):
            #print("PLAYER 2 WINS")
            return 0
        #check for draw
        elif((pl1 | pl2).bit_count() == ROWS*COLS):
            #print("DRAW")
            return 0.5

        turn *= -1
        pass
    pass

def play_model(model, initial_depth):
    pl1 = 0
    pl2 = 0
    turn = (-1) ** (rand.randint(0,8))
    while(True):
        legal_moves = moves_available(pl1, pl2)
        print("MOVES: " + str(legal_moves))
        depth = initial_depth + int((pl1 | pl2).bit_count()*0.1)
        print("DEPTH: " + str(depth))
        if(turn == 1):
            c = user_query(pl1, pl2)
            print(c)
            pl1 += get_move(c, pl1, pl2)
        else:
            c = minimax(pl2, pl1, model, depth)
            print(c)
            pl2 += get_move(c[1], pl2, pl1)

        print_board(pl1, pl2)
        #print(fill_feat_vec(pl1,pl2))

        #check for pl1 win
        if(turn == 1 and check_win(pl1)):
            print("USER WINS")
            return 
        #check for pl2 win
        elif(check_win(pl2)):
            print("MODEL WINS")
            return 
        #check for draw
        elif((pl1 | pl2).bit_count() == ROWS*COLS):
            print("DRAW")
            return 

        turn *= -1
        pass
    pass

# FEATURES: 
# 4 in a row
# 3 in a row unblocked 
# 3 in a row threats 
# dual threats 
# doubles unblocked 
# each row [0,7]
# each col [0,6]
NUM_FEATURES = 36
def fill_feat_vec(player, opponent):

    x = []

    # wins 
    my_wins = wins(player).bit_count()
    op_wins = wins(opponent).bit_count()
    
    #3 in a rows 
    my_possible_3s = 0
    op_possible_3s = 0
    directions = [1, 7]
    for d in directions:
        my_OOO_ = (player << d) & (player << 2*d) & (player << 3*d)
        my_OO_O = (player << d) & (player << 2*d) & (player >> d)
        my_O_OO = (player << d) & (player >> d) & (player >> 2*d)
        my__OOO = (player >> d) & (player >> 2*d) & (player >> 3*d)

        my_open_seq = (my_OOO_ | my_OO_O | my_O_OO | my__OOO) & (~ROW_MASKS[6])
        my_possible_3s = my_possible_3s | my_open_seq


        op_OOO_ = (opponent << d) & (opponent << 2*d) & (opponent << 3*d)
        op_OO_O = (opponent << d) & (opponent << 2*d) & (opponent >> d)
        op_O_OO = (opponent << d) & (opponent >> d) & (opponent >> 2*d)
        op__OOO = (opponent >> d) & (opponent >> 2*d) & (opponent >> 3*d)

        op_open_seq = (op_OOO_ | op_OO_O | op_O_OO | op__OOO) & (~ROW_MASKS[6])
        op_possible_3s = op_possible_3s | op_open_seq

    open_spaces = sum(0b0111111 << 7*c for c in range(COLS)) & ~(player | opponent)
    immediately_open_spaces = 0
    for c in range(COLS):
        if(move_available(c, player, opponent)):
            immediately_open_spaces = immediately_open_spaces | get_move(c, player, opponent)

    # open 3 in a rows 
    my_open_3s = (my_possible_3s & open_spaces).bit_count()
    op_open_3s = (op_possible_3s & open_spaces).bit_count()

    # immediate threats 
    my_threats = (my_possible_3s & immediately_open_spaces).bit_count()
    op_threats = (op_possible_3s & immediately_open_spaces).bit_count()

    # open double win spaces 
    P = (my_possible_3s & open_spaces)
    O = (op_possible_3s & open_spaces)
    my_open_doubles = 0
    op_open_doubles = 0
    for d in directions:
        my_open_doubles = my_open_doubles | (P & (P >> d))
        op_open_doubles = op_open_doubles | (O & (O >> d))
    my_open_doubles = my_open_doubles.bit_count()
    op_open_doubles = op_open_doubles.bit_count()

    # double threats 
    P = (my_possible_3s & immediately_open_spaces)
    O = (op_possible_3s & immediately_open_spaces)
    my_double_threats = 0
    op_double_threats = 0
    for d in directions:
        my_double_threats = my_double_threats | (P & (P << d))
        op_double_threats = op_double_threats | (O & (O << d))
    my_double_threats = my_double_threats.bit_count()
    op_double_threats = op_double_threats.bit_count()

    # rows and cols
    my_cols = []
    op_cols = []
    my_rows = []
    op_rows = []
    for i in range(COLS):
        my_cols.append((player & COL_MASKS[i]).bit_count())
        op_cols.append((opponent & COL_MASKS[i]).bit_count())
        my_rows.append((player & ROW_MASKS[i]).bit_count())
        op_rows.append((opponent & ROW_MASKS[i]).bit_count())   

    x.append(my_wins)
    x.append(my_open_3s)
    x.append(my_threats)
    x.append(my_open_doubles)
    x.append(my_double_threats)
    x.extend(my_cols)
    x.extend(my_rows[:6])

    x.append(op_wins)
    x.append(op_open_3s)
    x.append(op_threats)
    x.append(op_open_doubles)
    x.append(op_double_threats)
    x.extend(op_cols)
    x.extend(op_rows[:6])

    return np.array(x)

def random_query(player, opponent):
    moves = moves_available(player, opponent)
    return moves[rand.randint(0,len(moves)-1)]

def user_query(player, opponent):
    move = int(input("YOUR MOVE (COL INDEX): "))
    if(move >= COLS):
        raise ValueError("COLUMN INDEX OUT OF RANGE")
    return move

n = 1/(1.1 * np.array([4, 6, 5, 3, 2, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 3, 6, 5, 4, 2, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7]))
def model_query(player, opponent, w):
    x = fill_feat_vec(player, opponent)
    x_normalized = x * n
    #print(x_normalized)
    return np.dot(w, x_normalized)

def minimax(player, opponent, w, depth, alpha=float('-inf'), beta=float('inf'), root=True):
    best_val = float('-inf')
    best_move = None
    moves = moves_available(player, opponent)

    for move in moves:
        if(depth == 0):
            value = model_query(player | move, opponent, w)
            if(value > best_val):
                best_val = value
                best_move = move
        else:
            P = player | get_move(move, player, opponent)
            value = None
            if(check_win(P)): 
                value = float('inf')
            elif((P | opponent).bit_count() == 49):
                value = 0
            else:
                value = -1*(minimax(opponent, P, w, depth-1, alpha=(-beta), beta=(-alpha), root=False)[0])
            
            if(value >= best_val):
                best_val = value
                best_move = move
            
            #alpha-beta pruning
            if(not root):
                alpha = max(alpha, value)
                if(alpha >= beta):
                    break
                

                
        pass
    return best_val, best_move

def new_maxes(m, w):
    for i in range(len(m)):
        if(w[i] > m[i]):
            m[i] = w[i]
    return m

maxes = np.array([4, 6, 5, 3, 2, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 3, 6, 5, 4, 2, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7])
sim = 0
for _ in range(sim):
    pl1 = 0
    pl2 = 0
    turn = (-1) ** (rand.randint(0,1))
    features = None
    while(True):
        #print(moves_available(pl1, pl2))
        if(turn == 1):
            c = random_query(pl1, pl2)
            pl1 += get_move(c, pl1, pl2)
            features = fill_feat_vec(pl1, pl2)
        else:
            c = random_query(pl2, pl1)
            pl2 += get_move(c, pl2, pl1)
            features = fill_feat_vec(pl2, pl1)

        #print_board(pl1, pl2)
        maxes = new_maxes(maxes, features)

        if(features[0] > 4):
            print_board(pl1, pl2)
            print(features)

        #check for pl1 win
        if(turn == 1 and check_win(pl1)):
            #print("PLAYER 1 WINS")
            break
        #check for pl2 win
        elif(check_win(pl2)):
            #print("PLAYER 2 WINS")
            break
        #check for draw
        elif((pl1 | pl2).bit_count() == ROWS*COLS):
            #print("DRAW")
            break

        turn *= -1
        pass


    pass
#print(1/maxes)

def backup_model(model_file, backup_file):
    m = np.load(model_file)
    np.save(backup_file, m)
    pass

def F(parent, child, depth, games, executor):
    # score = 0
    # for _ in range(games):
    #     s = simulate_game(child, parent, depth)
    #     score += s
    #     print(f"{_+1} / {games}: {s}")
    # return score / games

    futures = [executor.submit(simulate_game, child, parent, depth, (-1)**g) for g in range(games)]
    results = [f.result() for f in futures]
    print(results)
    return sum(results) / games 

def train(parent_model_file, sigma, alpha, num_children, depth, eval_games, generations):
    parent_model = np.load(parent_model_file)
    training_data = np.load("training_data.npy")
    for g in range(generations):
        start = time.time()
        print("GENERATION " + str(g+1) + "...")

        #create epsilon
        epsilon_half = np.random.randn(num_children, NUM_FEATURES)
        epsilons = np.vstack([epsilon_half, -epsilon_half])

        #evaluate each epsilon
        F_ = []
        i = 1
        with ProcessPoolExecutor(max_workers=10) as executor:
            for epsilon in epsilons:
                score = F(parent_model, parent_model+sigma*epsilon, depth, eval_games, executor)
                F_.append(score)
                print(f"CHILD {i}: {score}")
                i += 1
                pass

        #normalize F sampling distribution
        mn = np.mean(F_)
        std = np.std(F_, ddof=1)

        mn = np.mean(np.array([0.5]))
        std = np.sqrt(sum((F_ - mn)**2)/(2*num_children-1))

        F_hat = (F_ - mn) / std

        #update weights
        grad_F = (1 / (2 * num_children * sigma)) * np.dot(F_hat, epsilons)
        if(std < 1e-8):
            grad_F = 0
        delta_w = alpha * grad_F
        parent_model = parent_model + delta_w
        
        #save new model and print update 
        np.save(parent_model_file, parent_model)
        total_time = time.time() - start
        print(f"TIME: {total_time}")
        print("dw: " + str(delta_w))
        print("MODEL: " + str(parent_model) + "\n")

        #evaluate new model
        print("EVAL: ")
        with ProcessPoolExecutor(max_workers=10) as executor:
            f = F(w, parent_model, 6, 50, executor)
            print(f)
            training_data = np.append(training_data, f)
            np.save("training_data.npy", training_data)     
        pass
    plt.plot(range(len(training_data)), training_data) 
    plt.show()
    pass

# w - baseline weights 
w = (1/n) * np.array([100000000,1000,1000,10000,100000,5,10,15,20,15,10,5,5,5,10,10,15,15,-100000000,-1000,-1000,-10000,-100000,-5,-10,-15,-20,-15,-10,-5,-5,-5,-10,-10,-15,-15])

# "connect4_weights.npy" - current model
if __name__ == "__main__":

    # play baseline model
    play_model(w, 8)

    # ###train
    # train(parent_model_file="connect4_weights.npy", 
    #       sigma=0.1,
    #       alpha=0.02,
    #       num_children=25,
    #       depth=5,
    #       eval_games=10,
    #       generations=10)

    ######### eval
    # print("EVAL: ")
    # with ProcessPoolExecutor(max_workers=10) as executor:
    #     print(F(w, np.load("connect4_weights.npy"), 6, 50, executor))

    # play trained model
    #play_model(np.load("connect4_weights.npy"), 8)

    # backup
    #backup_model("connect4_weights.npy", "connect4_weights1.npy")

    # load
    #backup_model("connect4_weights1.npy", "connect4_weights.npy")

    # y = np.load("training_data.npy")
    # x = range(len(y))
    # plt.plot(x, y)
    # plt.show()

    
    
    pass




